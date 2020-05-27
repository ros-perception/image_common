/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  Copyright (c) 2018, Open Robotics
*
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "camera_calibration_parsers/parse_ini.hpp"

#include <cassert>
#include <cctype>
#include <cmath>

#include <algorithm>
#include <array>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "sensor_msgs/distortion_models.hpp"

namespace camera_calibration_parsers
{

static rclcpp::Logger kIniLogger = rclcpp::get_logger("camera_calibration_parsers");

struct SimpleMatrix
{
  int rows;
  int cols;
  const double * data;

  SimpleMatrix(int rows, int cols, const double * data)
  : rows(rows), cols(cols), data(data)
  {}
};

std::ostream & operator<<(std::ostream & out, const SimpleMatrix & m)
{
  for (int i = 0; i < m.rows; ++i) {
    for (int j = 0; j < m.cols; ++j) {
      out << m.data[m.cols * i + j] << " ";
    }
    out << std::endl;
  }
  return out;
}

// Remove whitespace from both ends of a string.
void trim(std::string & s)
{
  s.erase(
    s.begin(), std::find_if(
      s.begin(), s.end(), [](int ch) {
        return !std::isspace(ch);
      }));
  s.erase(
    std::find_if(
      s.rbegin(), s.rend(), [](int ch) {
        return !std::isspace(ch);
      }).base(), s.end());
}

// Determine if a given string is an INI section header
bool is_section(const std::string & line)
{
  return line.find('[') != std::string::npos && line.find(']') != std::string::npos;
}

// Split input into lines.
std::vector<std::string> split_lines(std::istream & input)
{
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(input, line)) {
    lines.push_back(line);
  }
  return lines;
}

// Split lines into sections
std::vector<std::vector<std::string>> split_sections(
  const std::vector<std::string> & lines)
{
  std::vector<std::vector<std::string>> sections;
  std::vector<std::string> section;
  for (size_t ii = 0; ii < lines.size(); ++ii) {
    std::string line = lines[ii];
    trim(line);

    // Skip empty lines.
    if (line.length() == 0) {
      continue;
    }

    // Skip lines that are comments
    if (line[0] == '#' || line[0] == ';') {
      continue;
    }

    if (is_section(line) && section.size() > 0) {
      sections.push_back(section);
      section.clear();
    }
    section.push_back(line);
  }
  // Push the remaining content
  if (section.size() > 0) {
    sections.push_back(section);
  }

  return sections;
}

template<size_t rows, size_t cols>
std::array<double, rows * cols> parse_matrix(std::vector<std::string>::const_iterator & begin)
{
  std::array<double, rows * cols> ret;
  for (size_t ii = 0; ii < rows; ++ii) {
    auto ss = std::stringstream(*(begin++));
    for (size_t jj = 0; jj < cols; ++jj) {
      double val = std::numeric_limits<double>::quiet_NaN();
      if (!ss.eof()) {
        ss >> val;
      }
      ret[ii * cols + jj] = val;
    }
  }
  return ret;
}

bool parse_image_section(const std::vector<std::string> & section, CameraInfo & cam_info)
{
  auto width = std::find(section.begin(), section.end(), "width");
  if (width == section.end()) {
    RCLCPP_ERROR(kIniLogger, "Failed to find key 'width' in section '[image]'");
    return false;
  }

  auto height = std::find(section.begin(), section.end(), "height");
  if (height == section.end()) {
    RCLCPP_ERROR(kIniLogger, "Failed to find key 'height' in section '[image]'");
    return false;
  }

  cam_info.width = stoi(*(++width));
  cam_info.height = stoi(*(++height));
  return true;
}

bool parse_camera_section(
  const std::vector<std::string> & section,
  std::string & camera_name,
  CameraInfo & cam_info)
{
  camera_name = std::string(section[0].begin() + 1, section[0].end() - 1);

  auto camera_matrix = std::find(section.begin(), section.end(), "camera matrix");
  if (camera_matrix == section.end()) {
    RCLCPP_ERROR(kIniLogger, "Failed to find key 'camera matrix' in camera section");
    return false;
  }

  auto distortion = std::find(section.begin(), section.end(), "distortion");
  if (distortion == section.end()) {
    RCLCPP_ERROR(kIniLogger, "Failed to find key 'distortion' in camera section");
    return false;
  }

  auto rectification = std::find(section.begin(), section.end(), "rectification");
  if (rectification == section.end()) {
    RCLCPP_ERROR(kIniLogger, "Failed to find key 'rectification' in camera section");
    return false;
  }

  auto projection = std::find(section.begin(), section.end(), "projection");
  if (projection == section.end()) {
    RCLCPP_ERROR(kIniLogger, "Failed to find key 'projection' in camera section");
    return false;
  }

  auto d = parse_matrix<1, 8>(++distortion);
  if (std::isnan(d[5])) {
    cam_info.d = std::vector<double>(d.begin(), d.begin() + 5);
    cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  } else {
    cam_info.d = std::vector<double>(d.begin(), d.end());
    cam_info.distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
  }

  auto array_has_nan = [](auto a) {
      return std::end(a) != std::find_if(
        std::begin(a), std::end(a), [](double v) {
          return std::isnan(v);
        });
    };

  cam_info.k = parse_matrix<3, 3>(++camera_matrix);
  if (array_has_nan(cam_info.k)) {
    RCLCPP_ERROR(kIniLogger, "Error parsing 'camera matrix', incorrect size");
    return false;
  }

  cam_info.r = parse_matrix<3, 3>(++rectification);
  if (array_has_nan(cam_info.r)) {
    RCLCPP_ERROR(kIniLogger, "Error parsing 'rectification', incorrect size");
    return false;
  }

  cam_info.p = parse_matrix<3, 4>(++projection);
  if (array_has_nan(cam_info.p)) {
    RCLCPP_ERROR(kIniLogger, "Error parsing 'projection', incorrect size");
    return false;
  }
  return true;
}


bool parse_externals_section(const std::vector<std::string> & section)
{
  auto translation = std::find(section.begin(), section.end(), "translation");
  if (translation == section.end()) {
    RCLCPP_ERROR(kIniLogger, "Failed to find key 'translation' in section '[externals]'");
    // Don't error, because nothing is done with this anyway
  }

  auto rotation = std::find(section.begin(), section.end(), "rotation");
  if (rotation == section.end()) {
    RCLCPP_ERROR(kIniLogger, "Failed to find key 'rotation' in section '[externals]'");
    // Don't error, because nothing is done with this anyway
  }

  return true;
}

bool writeCalibrationIni(
  std::ostream & out, const std::string & camera_name,
  const CameraInfo & cam_info)
{
  // Videre INI format is legacy, only supports plumb bob distortion model.
  if (cam_info.distortion_model != sensor_msgs::distortion_models::PLUMB_BOB ||
    cam_info.d.size() != 5)
  {
    RCLCPP_ERROR(
      kIniLogger,
      "Videre INI format can only save calibrations using the plumb bob distortion model. "
      "Use the YAML format instead.\n"
      "\tdistortion_model = '%s', expected '%s'\n"
      "\tD.size() = %d, expected 5",
      cam_info.distortion_model.c_str(),
      sensor_msgs::distortion_models::PLUMB_BOB,
      static_cast<int>(cam_info.d.size()));
    return false;
  }

  out.precision(5);
  out << std::fixed;

  out << "# Camera intrinsics\n\n";
  /// @todo time?
  out << "[image]\n\n";
  out << "width\n" << cam_info.width << "\n\n";
  out << "height\n" << cam_info.height << "\n\n";
  out << "[" << camera_name << "]\n\n";

  out << "camera matrix\n" << SimpleMatrix(3, 3, &cam_info.k[0]);
  out << "\ndistortion\n" << SimpleMatrix(1, 5, &cam_info.d[0]);
  out << "\n\nrectification\n" << SimpleMatrix(3, 3, &cam_info.r[0]);
  out << "\nprojection\n" << SimpleMatrix(3, 4, &cam_info.p[0]);

  return true;
}

bool writeCalibrationIni(
  const std::string & file_name, const std::string & camera_name,
  const CameraInfo & cam_info)
{
  rcpputils::fs::path dir(rcpputils::fs::path(file_name).parent_path());
  if (!dir.empty() && !rcpputils::fs::exists(dir) &&
    !rcpputils::fs::create_directories(dir))
  {
    RCLCPP_ERROR(
      kIniLogger, "Unable to create directory for camera calibration file [%s]",
      dir.string().c_str());
    return false;
  }
  std::ofstream out(file_name);
  if (!out.is_open()) {
    RCLCPP_ERROR(
      kIniLogger, "Unable to open camera calibration file [%s] for writing",
      file_name.c_str());
    return false;
  }
  return writeCalibrationIni(out, camera_name, cam_info);
}


bool readCalibrationIni(
  std::istream & in, std::string & camera_name,
  CameraInfo & cam_info)
{
  auto lines = split_lines(in);
  if (lines.size() == 0) {
    RCLCPP_ERROR(kIniLogger, "Failed to detect content in .ini file");
    return false;
  }

  auto sections = split_sections(lines);
  if (sections.size() == 0) {
    RCLCPP_ERROR(kIniLogger, "Failed to detect valid sections in .ini file");
    return false;
  }

  for (auto section : sections) {
    if (section[0] == "[image]") {
      if (!parse_image_section(section, cam_info)) {
        return false;
      }
    } else if (section[0] == "[externals]") {
      if (!parse_externals_section(section)) {
        return false;
      }
    } else {
      if (!parse_camera_section(section, camera_name, cam_info)) {
        return false;
      }
    }
  }
  return true;
}

bool readCalibrationIni(
  const std::string & file_name, std::string & camera_name,
  CameraInfo & cam_info)
{
  std::fstream fs(file_name);
  return readCalibrationIni(fs, camera_name, cam_info);
}

bool parseCalibrationIni(
  const std::string & buffer, std::string & camera_name,
  CameraInfo & cam_info)
{
  std::stringstream ss(buffer);
  return readCalibrationIni(ss, camera_name, cam_info);
}

}  // namespace camera_calibration_parsers

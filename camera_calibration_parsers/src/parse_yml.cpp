/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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

#include "camera_calibration_parsers/parse_yml.hpp"

#include <cassert>
#include <cstring>
#include <ctime>

#include <fstream>
#include <string>

#include "rclcpp/logging.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "sensor_msgs/distortion_models.hpp"

#ifdef _WIN32
#define YAML_CPP_DLL
// TODO(mjcarroll): This shouldn't be needed, but there are some issues
// with MSVC and YAML-CPP that are upstream causing warnings in CI.
#pragma warning(push)
#pragma warning(disable: 4251)
#pragma warning(disable: 4275)
#include "yaml-cpp/yaml.h"
#pragma warning(pop)
#else
#include "yaml-cpp/yaml.h"
#endif

namespace camera_calibration_parsers
{

static rclcpp::Logger kYmlLogger = rclcpp::get_logger("camera_calibration_parsers");

/// \cond

static const char CAM_YML_NAME[] = "camera_name";
static const char WIDTH_YML_NAME[] = "image_width";
static const char HEIGHT_YML_NAME[] = "image_height";
static const char K_YML_NAME[] = "camera_matrix";
static const char D_YML_NAME[] = "distortion_coefficients";
static const char R_YML_NAME[] = "rectification_matrix";
static const char P_YML_NAME[] = "projection_matrix";
static const char DMODEL_YML_NAME[] = "distortion_model";

struct SimpleMatrix
{
  int rows;
  int cols;
  double * data;

  SimpleMatrix(int rows, int cols, double * data)
  : rows(rows), cols(cols), data(data)
  {}
};

YAML::Emitter & operator<<(YAML::Emitter & out, const SimpleMatrix & m)
{
  out << YAML::BeginMap;
  out << YAML::Key << "rows" << YAML::Value << m.rows;
  out << YAML::Key << "cols" << YAML::Value << m.cols;
  // out << YAML::Key << "dt"   << YAML::Value << "d"; // OpenCV data type specifier
  out << YAML::Key << "data" << YAML::Value;
  out << YAML::Flow;
  out << YAML::BeginSeq;
  for (int i = 0; i < m.rows * m.cols; ++i) {
    out << m.data[i];
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;
  return out;
}

template<typename T>
void operator>>(const YAML::Node & node, T & i)
{
  i = node.as<T>();
}

void operator>>(const YAML::Node & node, SimpleMatrix & m)
{
  int rows, cols;
  node["rows"] >> rows;
  assert(rows == m.rows);
  node["cols"] >> cols;
  assert(cols == m.cols);
  const YAML::Node & data = node["data"];
  for (int i = 0; i < rows * cols; ++i) {
    data[i] >> m.data[i];
  }
}

/// \endcond

bool writeCalibrationYml(
  std::ostream & out, const std::string & camera_name,
  const CameraInfo & cam_info)
{
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;

#if 0
  // Calibration time
  /// @todo Emitting the time breaks yaml-cpp on reading for some reason
  time_t raw_time;
  time(&raw_time);
  emitter << YAML::Key << "calibration_time";
  emitter << YAML::Value << asctime_r(localtime_r(&raw_time));
#endif

  // Image dimensions
  emitter << YAML::Key << WIDTH_YML_NAME << YAML::Value << static_cast<int>(cam_info.width);
  emitter << YAML::Key << HEIGHT_YML_NAME << YAML::Value << static_cast<int>(cam_info.height);

  // Camera name and intrinsics
  emitter << YAML::Key << CAM_YML_NAME << YAML::Value << camera_name;
  emitter << YAML::Key << K_YML_NAME << YAML::Value <<
    SimpleMatrix(3, 3, const_cast<double *>(&cam_info.k[0]));
  emitter << YAML::Key << DMODEL_YML_NAME << YAML::Value << cam_info.distortion_model;
  emitter << YAML::Key << D_YML_NAME << YAML::Value << SimpleMatrix(
    1,
    static_cast<int>(cam_info.d.size()),
    const_cast<double *>(&cam_info.d[0]));
  emitter << YAML::Key << R_YML_NAME << YAML::Value <<
    SimpleMatrix(3, 3, const_cast<double *>(&cam_info.r[0]));
  emitter << YAML::Key << P_YML_NAME << YAML::Value <<
    SimpleMatrix(3, 4, const_cast<double *>(&cam_info.p[0]));

  emitter << YAML::EndMap;

  out << emitter.c_str();
  return true;
}

bool writeCalibrationYml(
  const std::string & file_name, const std::string & camera_name,
  const CameraInfo & cam_info)
{
  rcpputils::fs::path dir(rcpputils::fs::path(file_name).parent_path());
  if (!dir.empty() && !rcpputils::fs::exists(dir) &&
    !rcpputils::fs::create_directories(dir))
  {
    RCLCPP_ERROR(
      kYmlLogger, "Unable to create directory for camera calibration file [%s]",
      dir.string().c_str());
  }
  std::ofstream out(file_name.c_str());
  if (!out.is_open()) {
    RCLCPP_ERROR(
      kYmlLogger, "Unable to open camera calibration file [%s] for writing",
      file_name.c_str());
    return false;
  }
  return writeCalibrationYml(out, camera_name, cam_info);
}

bool readCalibrationYml(
  std::istream & in, std::string & camera_name,
  CameraInfo & cam_info)
{
  try {
    YAML::Node doc = YAML::Load(in);

    if (doc[CAM_YML_NAME]) {
      doc[CAM_YML_NAME] >> camera_name;
    } else {
      camera_name = "unknown";
    }

    doc[WIDTH_YML_NAME] >> cam_info.width;
    doc[HEIGHT_YML_NAME] >> cam_info.height;

    // Read in fixed-size matrices
    SimpleMatrix K_(3, 3, &cam_info.k[0]);
    doc[K_YML_NAME] >> K_;
    SimpleMatrix R_(3, 3, &cam_info.r[0]);
    doc[R_YML_NAME] >> R_;
    SimpleMatrix P_(3, 4, &cam_info.p[0]);
    doc[P_YML_NAME] >> P_;

    // Different distortion models may have different numbers of parameters
    if (doc[DMODEL_YML_NAME]) {
      doc[DMODEL_YML_NAME] >> cam_info.distortion_model;
    } else {
      // Assume plumb bob for backwards compatibility
      cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
      RCLCPP_WARN(
        kYmlLogger,
        "Camera calibration file did not specify distortion model, assuming plumb bob");
    }
    const YAML::Node & D_node = doc[D_YML_NAME];
    int D_rows, D_cols;
    D_node["rows"] >> D_rows;
    D_node["cols"] >> D_cols;
    const YAML::Node & D_data = D_node["data"];
    cam_info.d.resize(D_rows * D_cols);
    for (int i = 0; i < D_rows * D_cols; ++i) {
      D_data[i] >> cam_info.d[i];
    }

    return true;
  } catch (YAML::Exception & e) {
    RCLCPP_WARN(kYmlLogger, "Exception parsing YAML camera calibration:\n%s", e.what());
    return false;
  }
}

bool readCalibrationYml(
  const std::string & file_name, std::string & camera_name,
  CameraInfo & cam_info)
{
  std::ifstream fin(file_name.c_str());
  if (!fin.good()) {
    RCLCPP_ERROR(kYmlLogger, "Unable to open camera calibration file [%s]", file_name.c_str());
    return false;
  }
  bool success = readCalibrationYml(fin, camera_name, cam_info);
  if (!success) {
    RCLCPP_ERROR(
      kYmlLogger, "Failed to parse camera calibration from file [%s]",
      file_name.c_str());
  }
  return success;
}

bool parseCalibrationYml(
  const std::string & buffer, std::string & camera_name,
  CameraInfo & cam_info)
{
  std::stringstream ss(buffer);
  return readCalibrationYml(ss, camera_name, cam_info);
}

}  // namespace camera_calibration_parsers

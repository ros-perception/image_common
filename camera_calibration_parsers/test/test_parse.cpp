// Copyright 2026 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <system_error>

#include "camera_calibration_parsers/parse.hpp"
#include "camera_calibration_parsers/parse_ini.hpp"
#include "camera_calibration_parsers/parse_yml.hpp"
#include "sensor_msgs/distortion_models.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "make_calibs.hpp"

using camera_calibration_parsers::parseCalibration;
using camera_calibration_parsers::parseCalibrationIni;
using camera_calibration_parsers::parseCalibrationYml;
using camera_calibration_parsers::readCalibration;
using camera_calibration_parsers::readCalibrationYml;
using camera_calibration_parsers::writeCalibration;
using camera_calibration_parsers::writeCalibrationYml;

class CalibrationFiles : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dir = std::filesystem::temp_directory_path() /
      ("camera_calibration_parsers_" + std::to_string(std::random_device{}()));
    std::filesystem::create_directories(dir);
  }

  void TearDown() override
  {
    std::error_code error;
    std::filesystem::remove_all(dir, error);
  }

  std::string path(const std::filesystem::path & relative) const
  {
    return (dir / relative).generic_string();
  }

  std::filesystem::path dir;
  const sensor_msgs::msg::CameraInfo calib = make_calib(sensor_msgs::distortion_models::PLUMB_BOB);
  std::string camera_name;
  sensor_msgs::msg::CameraInfo cam_info;
};

TEST_F(CalibrationFiles, roundtrip_every_format) {
  for (const std::string file : {"calib.ini", "calib.yml", "calib.yaml"}) {
    ASSERT_TRUE(writeCalibration(path(file), "my_camera", calib)) << file;
    ASSERT_TRUE(readCalibration(path(file), camera_name, cam_info)) << file;
    EXPECT_EQ(camera_name, "my_camera") << file;
    check_calib(cam_info);
  }
}

TEST_F(CalibrationFiles, unknown_extension) {
  EXPECT_FALSE(writeCalibration(path("calib.txt"), "my_camera", calib));
  EXPECT_FALSE(std::filesystem::exists(path("calib.txt")));

  std::ofstream(path("calib.txt")) << "image_width: 640\n";
  EXPECT_FALSE(readCalibration(path("calib.txt"), camera_name, cam_info));
}

TEST_F(CalibrationFiles, read_missing_file) {
  for (const std::string file : {"missing.ini", "missing.yml", "missing.yaml"}) {
    EXPECT_FALSE(readCalibration(path(file), camera_name, cam_info)) << file;
  }
}

TEST_F(CalibrationFiles, write_creates_directories) {
  for (const std::string file : {"a/b/calib.ini", "c/d/calib.yaml"}) {
    EXPECT_TRUE(writeCalibration(path(file), "my_camera", calib)) << file;
    EXPECT_TRUE(std::filesystem::exists(path(file))) << file;
  }
}

TEST_F(CalibrationFiles, write_cannot_create_directory) {
  std::ofstream(path("file")).close();
  for (const std::string file : {"file/sub/calib.ini", "file/sub/calib.yaml"}) {
    EXPECT_FALSE(writeCalibration(path(file), "my_camera", calib)) << file;
  }
}

TEST_F(CalibrationFiles, write_cannot_open_file) {
  for (const std::string file : {"calib.ini", "calib.yaml"}) {
    std::filesystem::create_directories(path(file));
    EXPECT_FALSE(writeCalibration(path(file), "my_camera", calib)) << file;
  }
}

TEST_F(CalibrationFiles, read_yml_file_that_cannot_be_parsed) {
  std::ofstream(path("calib.yaml")) << "image_width: [\n";
  EXPECT_FALSE(readCalibrationYml(path("calib.yaml"), camera_name, cam_info));
}

class ParseCalibration : public ::testing::Test
{
protected:
  std::string camera_name;
  sensor_msgs::msg::CameraInfo cam_info;
};

static const char * kIniImage = "[image]\nwidth\n640\nheight\n480\n";
static const char * kIniHeader = "[camera]\n";
static const char * kIniCameraMatrix = "camera matrix\n1 2 3\n4 5 6\n7 8 9\n";
static const char * kIniDistortion = "distortion\n1 2 3 4 5\n";
static const char * kIniRectification = "rectification\n1 0 0\n0 1 0\n0 0 1\n";
static const char * kIniProjection = "projection\n1 2 3 4\n5 6 7 8\n9 10 11 12\n";

static const char * kYmlMatrices =
  R"(
image_width: 640
image_height: 480
camera_matrix:
  rows: 3
  cols: 3
  data: [1, 2, 3, 4, 5, 6, 7, 8, 9]
distortion_coefficients:
  rows: 1
  cols: 5
  data: [1, 2, 3, 4, 5]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
)";

static std::string valid_ini()
{
  return std::string(kIniImage) + kIniHeader + kIniCameraMatrix + kIniDistortion +
         kIniRectification + kIniProjection;
}

TEST_F(ParseCalibration, every_format) {
  ASSERT_TRUE(parseCalibration(valid_ini(), "ini", camera_name, cam_info));
  EXPECT_EQ(camera_name, "camera");
  check_calib(cam_info);

  cam_info = sensor_msgs::msg::CameraInfo();
  const std::string yml = std::string("camera_name: from_yml\n") + kYmlMatrices;
  ASSERT_TRUE(parseCalibration(yml, "yml", camera_name, cam_info));
  EXPECT_EQ(camera_name, "from_yml");
  EXPECT_EQ(cam_info.k[8], 9.0);
}

TEST_F(ParseCalibration, unknown_format) {
  EXPECT_FALSE(parseCalibration(valid_ini(), "xml", camera_name, cam_info));
  EXPECT_FALSE(parseCalibration(valid_ini(), "", camera_name, cam_info));
}

TEST_F(ParseCalibration, ini_without_content) {
  EXPECT_FALSE(parseCalibrationIni("", camera_name, cam_info));
  EXPECT_FALSE(parseCalibrationIni("# comment\n\n; comment\n", camera_name, cam_info));
}

TEST_F(ParseCalibration, ini_image_section_missing_key) {
  EXPECT_FALSE(parseCalibrationIni("[image]\nheight\n480\n", camera_name, cam_info));
  EXPECT_FALSE(parseCalibrationIni("[image]\nwidth\n640\n", camera_name, cam_info));
}

TEST_F(ParseCalibration, ini_camera_section_missing_key) {
  const std::string header(kIniHeader);
  EXPECT_FALSE(
    parseCalibrationIni(
      header + kIniDistortion + kIniRectification + kIniProjection, camera_name, cam_info));
  EXPECT_FALSE(
    parseCalibrationIni(
      header + kIniCameraMatrix + kIniRectification + kIniProjection, camera_name, cam_info));
  EXPECT_FALSE(
    parseCalibrationIni(
      header + kIniCameraMatrix + kIniDistortion + kIniProjection, camera_name, cam_info));
  EXPECT_FALSE(
    parseCalibrationIni(
      header + kIniCameraMatrix + kIniDistortion + kIniRectification, camera_name, cam_info));
}

TEST_F(ParseCalibration, ini_matrix_row_too_short) {
  const std::string header(kIniHeader);
  EXPECT_FALSE(
    parseCalibrationIni(
      header + kIniCameraMatrix + kIniDistortion + kIniProjection +
      "rectification\n1 0\n0 1 0\n0 0 1\n", camera_name, cam_info));
  EXPECT_FALSE(
    parseCalibrationIni(
      header + kIniCameraMatrix + kIniDistortion + kIniRectification +
      "projection\n1 2 3\n5 6 7 8\n9 10 11 12\n", camera_name, cam_info));
}

TEST_F(ParseCalibration, ini_externals_keys_are_optional) {
  for (const std::string externals : {"[externals]\n", "[externals]\ntranslation\n1 2 3\n",
      "[externals]\nrotation\n1 2 3\n", "[externals]\ntranslation\n1 2 3\nrotation\n1 2 3\n"})
  {
    ASSERT_TRUE(parseCalibrationIni(valid_ini() + externals, camera_name, cam_info)) << externals;
    check_calib(cam_info);
  }
}

TEST_F(ParseCalibration, yml_defaults) {
  ASSERT_TRUE(parseCalibrationYml(kYmlMatrices, camera_name, cam_info));
  EXPECT_EQ(camera_name, "unknown");
  EXPECT_EQ(cam_info.distortion_model, sensor_msgs::distortion_models::PLUMB_BOB);
}

TEST_F(ParseCalibration, yml_empty_distortion_roundtrip) {
  sensor_msgs::msg::CameraInfo uncalibrated = make_calib("");
  ASSERT_TRUE(uncalibrated.d.empty());

  std::stringstream yml;
  ASSERT_TRUE(writeCalibrationYml(yml, "my_camera", uncalibrated));
  ASSERT_TRUE(parseCalibrationYml(yml.str(), camera_name, cam_info));
  EXPECT_EQ(camera_name, "my_camera");
  EXPECT_TRUE(cam_info.d.empty());
}

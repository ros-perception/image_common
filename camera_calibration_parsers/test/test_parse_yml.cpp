// Copyright 2018 Open Source Robotics Foundation, Inc.
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
//
#include <gtest/gtest.h>

#include <cstdio>
#include <cstdlib>
#include <string>

#include "camera_calibration_parsers/parse_yml.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "sensor_msgs/distortion_models.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "make_calibs.hpp"

std::string custom_tmpnam()
{
#ifdef _WIN32
  char name[L_tmpnam_s];
  errno_t err = tmpnam_s(name, L_tmpnam_s);
  if (err) {
    printf("Error occured creating unique filename.\n");
  }
  return std::string(name);
#else
  char temp[] = "/tmp/calib.XXXXXX";
  int fd = mkstemp(temp);
  close(fd);
  return std::string(temp);
#endif
}

static const char * kValidCalib5 =
  R"(
image_width: 640
image_height: 480
camera_name: mono_left
camera_matrix:
  rows: 3
  cols: 3
  data: [1, 2, 3, 4, 5, 6, 7, 8, 9]
distortion_model: plumb_bob
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

TEST(ParseYml, parse_valid_yml_calib5) {
  std::string camera_name;
  sensor_msgs::msg::CameraInfo cam_info;
  auto ret = camera_calibration_parsers::parseCalibrationYml(kValidCalib5, camera_name, cam_info);
  ASSERT_EQ(ret, true);
  ASSERT_EQ(camera_name, "mono_left");
  check_calib(cam_info);
}

static const char * kValidCalib8 =
  R"(
image_width: 640
image_height: 480
camera_name: mono_left
camera_matrix:
  rows: 3
  cols: 3
  data: [1, 2, 3, 4, 5, 6, 7, 8, 9]
distortion_model: rational_polynomial
distortion_coefficients:
  rows: 1
  cols: 8
  data: [1, 2, 3, 4, 5, 6, 7, 8]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
)";

TEST(ParseYml, parse_valid_yml_calib8) {
  std::string camera_name;
  sensor_msgs::msg::CameraInfo cam_info;
  auto ret = camera_calibration_parsers::parseCalibrationYml(kValidCalib8, camera_name, cam_info);
  ASSERT_EQ(ret, true);
  ASSERT_EQ(camera_name, "mono_left");
  check_calib(cam_info);
}

static const char * kInvalidCalib5 =
  R"(
image_width: 640
image_height: 480
camera_name: mono_left
#camera_matrix:
#  rows: 3
#  cols: 3
#  data: [1, 2, 3, 4, 5, 6, 7, 8, 9]
distortion_model: plumb_bob
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

TEST(ParseYml, parse_invalid_yml_calib5) {
  std::string camera_name;
  sensor_msgs::msg::CameraInfo cam_info;
  auto ret = camera_calibration_parsers::parseCalibrationYml(kInvalidCalib5, camera_name, cam_info);
  ASSERT_EQ(ret, false);
}

TEST(ParseYml, roundtrip_calib5) {
  std::string calib_file = custom_tmpnam();

  std::string camera_name = "roundtrip_calib5";
  auto cam_info = make_calib(sensor_msgs::distortion_models::PLUMB_BOB);
  auto ret_write = camera_calibration_parsers::writeCalibrationYml(
    calib_file, camera_name, cam_info);
  ASSERT_EQ(ret_write, true);

  std::string camera_name2;
  sensor_msgs::msg::CameraInfo cam_info2;
  auto ret_read = camera_calibration_parsers::readCalibrationYml(
    calib_file, camera_name2, cam_info2);
  ASSERT_EQ(ret_read, true);
  ASSERT_EQ(camera_name2, camera_name);
  check_calib(cam_info2);
}

TEST(ParseYml, roundtrip_calib8) {
  std::string calib_file = custom_tmpnam();

  std::string camera_name = "roundtrip_calib8";
  auto cam_info = make_calib(sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL);
  auto ret_write = camera_calibration_parsers::writeCalibrationYml(
    calib_file, camera_name, cam_info);
  ASSERT_EQ(ret_write, true);

  std::string camera_name2;
  sensor_msgs::msg::CameraInfo cam_info2;
  auto ret_read = camera_calibration_parsers::readCalibrationYml(
    calib_file, camera_name2, cam_info2);

  ASSERT_EQ(ret_read, true);
  ASSERT_EQ(camera_name2, camera_name);
  check_calib(cam_info2);
}

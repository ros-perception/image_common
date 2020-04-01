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
#include <string>

#include "camera_calibration_parsers/parse_ini.hpp"
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
# Comment with pound
; Comment with semicolon
[image]

width
640

height
480

[externals]

translation
0.0 0.0 0.0

rotation
0.0 0.0 0.0

[mono_left]

camera matrix
1 2 3
4 5 6
7 8 9

distortion
1 2 3 4 5

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
1.0  2.0  3.0  4.0
5.0  6.0  7.0  8.0
9.0 10.0 11.0 12.0
)";

TEST(ParseIni, parse_valid_ini_calib5) {
  std::string camera_name;
  sensor_msgs::msg::CameraInfo cam_info;
  auto ret = camera_calibration_parsers::parseCalibrationIni(kValidCalib5, camera_name, cam_info);
  ASSERT_EQ(ret, true);
  ASSERT_EQ(camera_name, "mono_left");
  check_calib(cam_info);
}

static const char * kValidCalib8 =
  R"(
# Comment with pound
; Comment with semicolon
[image]

width
640

height
480

[externals]

translation
0.0 0.0 0.0

rotation
0.0 0.0 0.0

[mono_left]

camera matrix
1 2 3
4 5 6
7 8 9

distortion
1 2 3 4 5 6 7 8

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
1.0  2.0  3.0  4.0
5.0  6.0  7.0  8.0
9.0 10.0 11.0 12.0
)";

TEST(ParseIni, parse_valid_ini_calib8) {
  std::string camera_name;
  sensor_msgs::msg::CameraInfo cam_info;
  auto ret = camera_calibration_parsers::parseCalibrationIni(kValidCalib8, camera_name, cam_info);
  ASSERT_EQ(ret, true);
  ASSERT_EQ(camera_name, "mono_left");
  check_calib(cam_info);
}

static const char * kInvalidCalib5 =
  R"(
# Comment with pound
; Comment with semicolon
[image]

width
640

height
480

[mono_left]

camera matrix
1 2 3
4 5 6
7 8

distortion
1 2 3 4 5

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
1.0  2.0  3.0  4.0
5.0  6.0  7.0  8.0
9.0 10.0 11.0 12.0
)";

TEST(ParseIni, parse_invalid_ini_calib5) {
  std::string camera_name;
  sensor_msgs::msg::CameraInfo cam_info;
  auto ret = camera_calibration_parsers::parseCalibrationIni(kInvalidCalib5, camera_name, cam_info);
  ASSERT_EQ(ret, false);
}

static const char * kInvalidCalib5_2 =
  R"(
# Comment with pound
; Comment with semicolon
[image]

width
640

height
480

[mono_left]

camera matrix
1 2 3
4 5 6

distortion
1 2 3 4 5

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
1.0  2.0  3.0  4.0
5.0  6.0  7.0  8.0
9.0 10.0 11.0 12.0
)";

TEST(ParseIni, parse_invalid_ini_calib5_2) {
  std::string camera_name;
  sensor_msgs::msg::CameraInfo cam_info;
  auto ret =
    camera_calibration_parsers::parseCalibrationIni(kInvalidCalib5_2, camera_name, cam_info);
  ASSERT_EQ(ret, false);
}

TEST(ParseIni, roundtrip_calib5) {
  std::string calib_file = custom_tmpnam();

  std::string camera_name = "roundtrip_calib5";
  auto cam_info = make_calib(sensor_msgs::distortion_models::PLUMB_BOB);
  auto ret_write =
    camera_calibration_parsers::writeCalibrationIni(calib_file, camera_name, cam_info);
  ASSERT_EQ(ret_write, true);

  std::string camera_name2;
  sensor_msgs::msg::CameraInfo cam_info2;
  auto ret_read =
    camera_calibration_parsers::readCalibrationIni(calib_file, camera_name2, cam_info2);
  ASSERT_EQ(ret_read, true);
  ASSERT_EQ(camera_name2, camera_name);
  check_calib(cam_info2);
}

TEST(ParseIni, cant_write_calib8) {
  std::string calib_file = custom_tmpnam();

  std::string camera_name = "roundtrip_calib8";
  auto cam_info = make_calib(sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL);
  auto ret_write =
    camera_calibration_parsers::writeCalibrationIni(calib_file, camera_name, cam_info);
  ASSERT_EQ(ret_write, false);
}

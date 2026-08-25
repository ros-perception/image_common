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
#include <fstream>
#include <string>
#include <utility>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/stat.h>
#endif

#include "camera_calibration_parsers/parse_ini.hpp"
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

class ScopedReadOnlyFile
{
public:
  explicit ScopedReadOnlyFile(std::string path)
  : path_(std::move(path))
  {}

  ScopedReadOnlyFile(const ScopedReadOnlyFile &) = delete;
  ScopedReadOnlyFile & operator=(const ScopedReadOnlyFile &) = delete;

  ~ScopedReadOnlyFile()
  {
#ifdef _WIN32
    if (original_attributes_ != INVALID_FILE_ATTRIBUTES) {
      SetFileAttributesA(path_.c_str(), original_attributes_);
    }
#else
    if (has_original_mode_) {
      chmod(path_.c_str(), original_mode_);
    }
#endif
    std::remove(path_.c_str());
  }

  bool make_read_only()
  {
#ifdef _WIN32
    original_attributes_ = GetFileAttributesA(path_.c_str());
    return original_attributes_ != INVALID_FILE_ATTRIBUTES &&
           SetFileAttributesA(path_.c_str(), original_attributes_ | FILE_ATTRIBUTE_READONLY) != 0;
#else
    struct stat file_info;
    has_original_mode_ = stat(path_.c_str(), &file_info) == 0;
    if (has_original_mode_) {
      original_mode_ = file_info.st_mode;
    }
    return has_original_mode_ &&
           chmod(path_.c_str(), S_IRUSR | S_IRGRP | S_IROTH) == 0;
#endif
  }

private:
  std::string path_;
#ifdef _WIN32
  DWORD original_attributes_{INVALID_FILE_ATTRIBUTES};
#else
  mode_t original_mode_{};
  bool has_original_mode_{false};
#endif
};

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

TEST(ParseIni, read_only_calib5) {
  std::string calib_file = custom_tmpnam();
  ScopedReadOnlyFile read_only_file(calib_file);

  std::string camera_name = "read_only_calib5";
  auto cam_info = make_calib(sensor_msgs::distortion_models::PLUMB_BOB);
  ASSERT_TRUE(
    camera_calibration_parsers::writeCalibrationIni(calib_file, camera_name, cam_info));

  ASSERT_TRUE(read_only_file.make_read_only());

  std::ifstream read_probe(calib_file);
  ASSERT_TRUE(read_probe.is_open());
  read_probe.close();

  std::fstream read_write_probe(calib_file);
  if (read_write_probe.is_open()) {
    GTEST_SKIP() << "Environment does not enforce read-only file permissions";
  }

  std::string camera_name2;
  sensor_msgs::msg::CameraInfo cam_info2;
  ASSERT_TRUE(
    camera_calibration_parsers::readCalibrationIni(calib_file, camera_name2, cam_info2));
  EXPECT_EQ(camera_name2, camera_name);
  check_calib(cam_info2);
}

TEST(ParseIni, cant_read_missing_file) {
  std::string calib_file = custom_tmpnam();
  std::remove(calib_file.c_str());

  std::string camera_name;
  sensor_msgs::msg::CameraInfo cam_info;
  ASSERT_FALSE(
    camera_calibration_parsers::readCalibrationIni(calib_file, camera_name, cam_info));
}

TEST(ParseIni, cant_write_calib8) {
  std::string calib_file = custom_tmpnam();

  std::string camera_name = "roundtrip_calib8";
  auto cam_info = make_calib(sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL);
  auto ret_write =
    camera_calibration_parsers::writeCalibrationIni(calib_file, camera_name, cam_info);
  ASSERT_EQ(ret_write, false);
}

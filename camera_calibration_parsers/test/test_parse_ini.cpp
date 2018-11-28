#include <gtest/gtest.h>

#include <sensor_msgs/msg/camera_info.hpp>

#include <camera_calibration_parsers/parse_ini.h>

std::string valid_calib5 =
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
  auto ret = camera_calibration_parsers::parseCalibrationIni(valid_calib5, camera_name, cam_info);
  ASSERT_EQ(ret, true);

  ASSERT_EQ(cam_info.width, 640U);
  ASSERT_EQ(cam_info.height, 480U);
  ASSERT_EQ(camera_name, "mono_left");

  ASSERT_DOUBLE_EQ(cam_info.k[0], 1.0);
  ASSERT_DOUBLE_EQ(cam_info.k[1], 2.0);
  ASSERT_DOUBLE_EQ(cam_info.k[2], 3.0);
  ASSERT_DOUBLE_EQ(cam_info.k[3], 4.0);
  ASSERT_DOUBLE_EQ(cam_info.k[4], 5.0);
  ASSERT_DOUBLE_EQ(cam_info.k[5], 6.0);
  ASSERT_DOUBLE_EQ(cam_info.k[6], 7.0);
  ASSERT_DOUBLE_EQ(cam_info.k[7], 8.0);
  ASSERT_DOUBLE_EQ(cam_info.k[8], 9.0);

  ASSERT_EQ(cam_info.distortion_model, "plumb_bob");
  ASSERT_EQ(cam_info.d.size(), 5U);
  ASSERT_DOUBLE_EQ(cam_info.d[0], 1);
  ASSERT_DOUBLE_EQ(cam_info.d[1], 2);
  ASSERT_DOUBLE_EQ(cam_info.d[2], 3);
  ASSERT_DOUBLE_EQ(cam_info.d[3], 4);
  ASSERT_DOUBLE_EQ(cam_info.d[4], 5);

  ASSERT_DOUBLE_EQ(cam_info.r[0], 1.0);
  ASSERT_DOUBLE_EQ(cam_info.r[1], 0.0);
  ASSERT_DOUBLE_EQ(cam_info.r[2], 0.0);
  ASSERT_DOUBLE_EQ(cam_info.r[3], 0.0);
  ASSERT_DOUBLE_EQ(cam_info.r[4], 1.0);
  ASSERT_DOUBLE_EQ(cam_info.r[5], 0.0);
  ASSERT_DOUBLE_EQ(cam_info.r[6], 0.0);
  ASSERT_DOUBLE_EQ(cam_info.r[7], 0.0);
  ASSERT_DOUBLE_EQ(cam_info.r[8], 1.0);

  ASSERT_DOUBLE_EQ(cam_info.p[0], 1.0);
  ASSERT_DOUBLE_EQ(cam_info.p[1], 2.0);
  ASSERT_DOUBLE_EQ(cam_info.p[2], 3.0);
  ASSERT_DOUBLE_EQ(cam_info.p[3], 4.0);
  ASSERT_DOUBLE_EQ(cam_info.p[4], 5.0);
  ASSERT_DOUBLE_EQ(cam_info.p[5], 6.0);
  ASSERT_DOUBLE_EQ(cam_info.p[6], 7.0);
  ASSERT_DOUBLE_EQ(cam_info.p[7], 8.0);
  ASSERT_DOUBLE_EQ(cam_info.p[8], 9.0);
  ASSERT_DOUBLE_EQ(cam_info.p[9], 10.0);
  ASSERT_DOUBLE_EQ(cam_info.p[10], 11.0);
  ASSERT_DOUBLE_EQ(cam_info.p[11], 12.0);
}

std::string valid_calib8 =
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
  auto ret = camera_calibration_parsers::parseCalibrationIni(valid_calib8, camera_name, cam_info);
  ASSERT_EQ(ret, true);

  ASSERT_EQ(cam_info.width, 640U);
  ASSERT_EQ(cam_info.height, 480U);
  ASSERT_EQ(camera_name, "mono_left");

  ASSERT_DOUBLE_EQ(cam_info.k[0], 1.0);
  ASSERT_DOUBLE_EQ(cam_info.k[1], 2.0);
  ASSERT_DOUBLE_EQ(cam_info.k[2], 3.0);
  ASSERT_DOUBLE_EQ(cam_info.k[3], 4.0);
  ASSERT_DOUBLE_EQ(cam_info.k[4], 5.0);
  ASSERT_DOUBLE_EQ(cam_info.k[5], 6.0);
  ASSERT_DOUBLE_EQ(cam_info.k[6], 7.0);
  ASSERT_DOUBLE_EQ(cam_info.k[7], 8.0);
  ASSERT_DOUBLE_EQ(cam_info.k[8], 9.0);

  ASSERT_EQ(cam_info.distortion_model, "rational_polynomial");
  ASSERT_EQ(cam_info.d.size(), 8U);
  ASSERT_DOUBLE_EQ(cam_info.d[0], 1);
  ASSERT_DOUBLE_EQ(cam_info.d[1], 2);
  ASSERT_DOUBLE_EQ(cam_info.d[2], 3);
  ASSERT_DOUBLE_EQ(cam_info.d[3], 4);
  ASSERT_DOUBLE_EQ(cam_info.d[4], 5);
  ASSERT_DOUBLE_EQ(cam_info.d[5], 6);
  ASSERT_DOUBLE_EQ(cam_info.d[6], 7);
  ASSERT_DOUBLE_EQ(cam_info.d[7], 8);

  ASSERT_DOUBLE_EQ(cam_info.d[5], 6);
  ASSERT_DOUBLE_EQ(cam_info.r[0], 1.0);
  ASSERT_DOUBLE_EQ(cam_info.r[1], 0.0);
  ASSERT_DOUBLE_EQ(cam_info.r[2], 0.0);
  ASSERT_DOUBLE_EQ(cam_info.r[3], 0.0);
  ASSERT_DOUBLE_EQ(cam_info.r[4], 1.0);
  ASSERT_DOUBLE_EQ(cam_info.r[5], 0.0);
  ASSERT_DOUBLE_EQ(cam_info.r[6], 0.0);
  ASSERT_DOUBLE_EQ(cam_info.r[7], 0.0);
  ASSERT_DOUBLE_EQ(cam_info.r[8], 1.0);

  ASSERT_DOUBLE_EQ(cam_info.p[0], 1.0);
  ASSERT_DOUBLE_EQ(cam_info.p[1], 2.0);
  ASSERT_DOUBLE_EQ(cam_info.p[2], 3.0);
  ASSERT_DOUBLE_EQ(cam_info.p[3], 4.0);
  ASSERT_DOUBLE_EQ(cam_info.p[4], 5.0);
  ASSERT_DOUBLE_EQ(cam_info.p[5], 6.0);
  ASSERT_DOUBLE_EQ(cam_info.p[6], 7.0);
  ASSERT_DOUBLE_EQ(cam_info.p[7], 8.0);
  ASSERT_DOUBLE_EQ(cam_info.p[8], 9.0);
  ASSERT_DOUBLE_EQ(cam_info.p[9], 10.0);
  ASSERT_DOUBLE_EQ(cam_info.p[10], 11.0);
  ASSERT_DOUBLE_EQ(cam_info.p[11], 12.0);
}

std::string invalid_calib5 =
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
  auto ret = camera_calibration_parsers::parseCalibrationIni(invalid_calib5, camera_name, cam_info);
  ASSERT_EQ(ret, false);
}

std::string invalid_calib5_2 =
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
  auto ret = camera_calibration_parsers::parseCalibrationIni(invalid_calib5, camera_name, cam_info);
  ASSERT_EQ(ret, false);
}


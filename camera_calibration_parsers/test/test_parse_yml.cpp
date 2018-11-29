#include <gtest/gtest.h>

#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <camera_calibration_parsers/parse_yml.h>
#include <camera_calibration_parsers/impl/filesystem_helper.hpp>

#include "make_calibs.hpp"

namespace fs = camera_calibration_parsers::impl::fs;

std::string valid_calib5 =
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
  auto ret = camera_calibration_parsers::parseCalibrationYml(valid_calib5, camera_name, cam_info);
  ASSERT_EQ(ret, true);
  ASSERT_EQ(camera_name, "mono_left");
  check_calib(cam_info);
}

std::string valid_calib8 =
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
  auto ret = camera_calibration_parsers::parseCalibrationYml(valid_calib8, camera_name, cam_info);
  ASSERT_EQ(ret, true);
  ASSERT_EQ(camera_name, "mono_left");
  check_calib(cam_info);
}

std::string invalid_calib5 =
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
  auto ret = camera_calibration_parsers::parseCalibrationYml(invalid_calib5, camera_name, cam_info);
  ASSERT_EQ(ret, false);
}

TEST(ParseYml, roundtrip_calib5) {
  auto tmpdir = fs::temp_directory_path();
  auto calib_file = tmpdir / "calib5.yml";

  std::string camera_name = "roundtrip_calib5";
  auto cam_info = make_calib(sensor_msgs::distortion_models::PLUMB_BOB);
  auto ret_write = camera_calibration_parsers::writeCalibrationYml(calib_file, camera_name, cam_info);
  ASSERT_EQ(ret_write, true);

  std::string camera_name2;
  sensor_msgs::msg::CameraInfo cam_info2;
  auto ret_read = camera_calibration_parsers::readCalibrationYml(calib_file, camera_name2, cam_info2);
  ASSERT_EQ(ret_read, true);
  ASSERT_EQ(camera_name2, camera_name);
  check_calib(cam_info2);
}

TEST(ParseYml, roundtrip_calib8) {
  auto tmpdir = fs::temp_directory_path();
  auto calib_file = tmpdir / "calib8.yml";

  std::string camera_name = "roundtrip_calib8";
  auto cam_info = make_calib(sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL);
  auto ret_write = camera_calibration_parsers::writeCalibrationYml(calib_file, camera_name, cam_info);
  ASSERT_EQ(ret_write, true);

  std::string camera_name2;
  sensor_msgs::msg::CameraInfo cam_info2;
  auto ret_read = camera_calibration_parsers::readCalibrationYml(calib_file, camera_name2, cam_info2);

  std::cerr << cam_info.distortion_model << " " << cam_info2.distortion_model << std::endl;

  ASSERT_EQ(ret_read, true);
  ASSERT_EQ(camera_name2, camera_name);
  check_calib(cam_info2);
}


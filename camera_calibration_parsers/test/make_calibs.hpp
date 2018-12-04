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
#ifndef MAKE_CALIBS_HPP__
#define MAKE_CALIBS_HPP__

#include <string>

#include "sensor_msgs/distortion_models.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

void check_calib(sensor_msgs::msg::CameraInfo cam_info)
{
  ASSERT_EQ(cam_info.width, 640U);
  ASSERT_EQ(cam_info.height, 480U);

  ASSERT_DOUBLE_EQ(cam_info.k[0], 1.0);
  ASSERT_DOUBLE_EQ(cam_info.k[1], 2.0);
  ASSERT_DOUBLE_EQ(cam_info.k[2], 3.0);
  ASSERT_DOUBLE_EQ(cam_info.k[3], 4.0);
  ASSERT_DOUBLE_EQ(cam_info.k[4], 5.0);
  ASSERT_DOUBLE_EQ(cam_info.k[5], 6.0);
  ASSERT_DOUBLE_EQ(cam_info.k[6], 7.0);
  ASSERT_DOUBLE_EQ(cam_info.k[7], 8.0);
  ASSERT_DOUBLE_EQ(cam_info.k[8], 9.0);

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

  if (cam_info.distortion_model == sensor_msgs::distortion_models::PLUMB_BOB) {
    ASSERT_EQ(cam_info.d.size(), 5U);
    ASSERT_DOUBLE_EQ(cam_info.d[0], 1);
    ASSERT_DOUBLE_EQ(cam_info.d[1], 2);
    ASSERT_DOUBLE_EQ(cam_info.d[2], 3);
    ASSERT_DOUBLE_EQ(cam_info.d[3], 4);
    ASSERT_DOUBLE_EQ(cam_info.d[4], 5);
  } else if (cam_info.distortion_model == sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL) {
    ASSERT_EQ(cam_info.d.size(), 8U);
    ASSERT_DOUBLE_EQ(cam_info.d[0], 1);
    ASSERT_DOUBLE_EQ(cam_info.d[1], 2);
    ASSERT_DOUBLE_EQ(cam_info.d[2], 3);
    ASSERT_DOUBLE_EQ(cam_info.d[3], 4);
    ASSERT_DOUBLE_EQ(cam_info.d[4], 5);
    ASSERT_DOUBLE_EQ(cam_info.d[5], 6);
    ASSERT_DOUBLE_EQ(cam_info.d[6], 7);
    ASSERT_DOUBLE_EQ(cam_info.d[7], 8);
  } else {
    ADD_FAILURE() << "Unknown camera distortion model " << cam_info.distortion_model;
  }
}

sensor_msgs::msg::CameraInfo make_calib(const std::string & distortion_model)
{
  sensor_msgs::msg::CameraInfo cam_info;
  cam_info.width = 640;
  cam_info.height = 480;

  cam_info.k = {{1, 2, 3, 4, 5, 6, 7, 8, 9}};
  cam_info.r = {{1, 0, 0, 0, 1, 0, 0, 0, 1}};
  cam_info.p = {{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}};

  if (distortion_model == sensor_msgs::distortion_models::PLUMB_BOB) {
    cam_info.d = {1, 2, 3, 4, 5};
    cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  } else if (distortion_model == sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL) {
    cam_info.d = {1, 2, 3, 4, 5, 6, 7, 8};
    cam_info.distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
  }
  return cam_info;
}

#endif  // MAKE_CALIBS_HPP__

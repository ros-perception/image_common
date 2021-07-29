// Copyright 2021, Open Source Robotics Foundation, Inc.
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

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <memory>

#include "image_transport_tutorial/resized_publisher.hpp"
#include "rclcpp/logging.hpp"

void ResizedPublisher::publish(
  const sensor_msgs::msg::Image & message,
  const PublishFn & publish_fn) const
{
  cv::Mat cv_image;
  std::shared_ptr<void const> tracked_object;
  try {
    cv_image = cv_bridge::toCvShare(message, tracked_object, message.encoding)->image;
  } catch (cv::Exception & e) {
    auto logger = rclcpp::get_logger("resized_publisher");
    RCLCPP_ERROR(
      logger, "Could not convert from '%s' to '%s'.",
      message.encoding.c_str(), message.encoding.c_str());
    return;
  }

  // Rescale image
  double subsampling_factor = 2.0;
  int new_width = cv_image.cols / subsampling_factor + 0.5;
  int new_height = cv_image.rows / subsampling_factor + 0.5;
  cv::Mat buffer;
  cv::resize(cv_image, buffer, cv::Size(new_width, new_height));

  // Set up ResizedImage and publish
  image_transport_tutorial::msg::ResizedImage resized_image;
  resized_image.original_height = cv_image.rows;
  resized_image.original_width = cv_image.cols;
  resized_image.image = *(cv_bridge::CvImage(message.header, "bgr8", cv_image).toImageMsg());
  publish_fn(resized_image);
}

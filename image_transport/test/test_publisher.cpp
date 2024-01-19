// Copyright (c) 2018 Open Source Robotics Foundation, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "image_transport/image_transport.hpp"

class TestPublisher : public ::testing::Test
{
protected:
  void SetUp()
  {
    node_ = rclcpp::Node::make_shared("test_publisher");
  }

  rclcpp::Node::SharedPtr node_;
};

TEST_F(TestPublisher, publisher) {
  auto pub = image_transport::create_publisher(node_.get(), "camera/image");
  EXPECT_EQ(node_->get_node_graph_interface()->count_publishers("camera/image"), 1u);
  pub.shutdown();
  EXPECT_EQ(node_->get_node_graph_interface()->count_publishers("camera/image"), 0u);
  // coverage tests: invalid publisher should fail but not crash
  pub.publish(sensor_msgs::msg::Image());
  pub.publish(sensor_msgs::msg::Image::ConstSharedPtr());
}

TEST_F(TestPublisher, image_transport_publisher) {
  image_transport::ImageTransport it(node_);
  auto pub = it.advertise("camera/image", 1);
}

TEST_F(TestPublisher, camera_publisher) {
  auto camera_pub = image_transport::create_camera_publisher(node_.get(), "camera/image");
  EXPECT_EQ(node_->get_node_graph_interface()->count_publishers("camera/image"), 1u);
  EXPECT_EQ(node_->get_node_graph_interface()->count_publishers("camera/camera_info"), 1u);
  camera_pub.shutdown();
  EXPECT_EQ(node_->get_node_graph_interface()->count_publishers("camera/image"), 0u);
  EXPECT_EQ(node_->get_node_graph_interface()->count_publishers("camera/camera_info"), 0u);
  // coverage tests: invalid publisher should fail but not crash
  camera_pub.publish(sensor_msgs::msg::Image(), sensor_msgs::msg::CameraInfo());
  camera_pub.publish(
    sensor_msgs::msg::Image::ConstSharedPtr(),
    sensor_msgs::msg::CameraInfo::ConstSharedPtr());
  sensor_msgs::msg::Image image;
  sensor_msgs::msg::CameraInfo info;
  camera_pub.publish(image, info, rclcpp::Time());
}

TEST_F(TestPublisher, image_transport_camera_publisher) {
  image_transport::ImageTransport it(node_);
  auto pub = it.advertiseCamera("camera/image", 1);
}

TEST_F(TestPublisher, image_transport_camera_publisher_qos) {
  image_transport::ImageTransport it(node_);
  auto pub = it.advertise("camera/image", rmw_qos_profile_sensor_data);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}

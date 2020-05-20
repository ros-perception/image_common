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

#include <chrono>
#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "utils.hpp"

#include "image_transport/image_transport.hpp"

class TestPublisher : public ::testing::Test
{
protected:
  void SetUp()
  {
    node_ = rclcpp::Node::make_shared("node", "namespace");
    std::vector<std::string> arguments{
      "--ros-args", "-r", "old_topic:=new_topic"
    };
    rclcpp::NodeOptions node_options;
    node_options.arguments(arguments);

    node_remap_ = rclcpp::Node::make_shared(
      "node_remap",
      "namespace",
      node_options
    );
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr node_remap_;
};

TEST_F(TestPublisher, RemappedPublisher) {
  const size_t max_retries = 3;
  const size_t max_loops = 200;
  const std::chrono::milliseconds sleep_per_loop = std::chrono::milliseconds(10);

  rclcpp::executors::SingleThreadedExecutor executor;
  auto image = std::make_shared<sensor_msgs::msg::Image>();

  // Subscribe
  bool received{false};
  auto sub = image_transport::create_subscription(
    node_remap_.get(), "old_topic",
    [&received](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
      (void)msg;
      received = true;
    }, "raw");

  // Publish
  auto pub = image_transport::create_publisher(node_.get(), "new_topic");

  ASSERT_EQ("/namespace/new_topic", sub.getTopic());
  ASSERT_EQ("/namespace/new_topic", pub.getTopic());

  test_rclcpp::wait_for_subscriber(node_, sub.getTopic());

  ASSERT_FALSE(received);

  size_t retry = 0;
  size_t nSub = 0;
  size_t nPub = 0;
  while (retry < max_retries && nPub == 0 && nSub == 0) {
    nSub = pub.getNumSubscribers();
    nPub = sub.getNumPublishers();
    std::this_thread::sleep_for(sleep_per_loop);
  }

  executor.spin_node_some(node_);
  executor.spin_node_some(node_remap_);

  retry = 0;
  while (retry < max_retries && !received) {
    // generate random image and publish it
    pub.publish(image);

    executor.spin_node_some(node_);
    executor.spin_node_some(node_remap_);

    size_t loop = 0;
    while ((!received) && (loop++ < max_loops)) {
      std::this_thread::sleep_for(sleep_per_loop);
      executor.spin_node_some(node_);
      executor.spin_node_some(node_remap_);
    }
  }

  EXPECT_TRUE(received);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}

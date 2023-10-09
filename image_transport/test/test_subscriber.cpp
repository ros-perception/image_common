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

class TestSubscriber : public ::testing::Test
{
protected:
  void SetUp()
  {
    node_ = rclcpp::Node::make_shared("test_subscriber");
  }

  rclcpp::Node::SharedPtr node_;
};

TEST_F(TestSubscriber, construction_and_destruction) {
  std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr & msg)> fcn =
    [](const auto & msg) {(void)msg;};

  auto sub = image_transport::create_subscription(node_.get(), "camera/image", fcn, "raw");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.spin_node_some(node_);
}

TEST_F(TestSubscriber, shutdown) {
  std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr & msg)> fcn =
    [](const auto & msg) {(void)msg;};

  auto sub = image_transport::create_subscription(node_.get(), "camera/image", fcn, "raw");
  EXPECT_EQ(node_->get_node_graph_interface()->count_subscribers("camera/image"), 1u);
  sub.shutdown();
  EXPECT_EQ(node_->get_node_graph_interface()->count_subscribers("camera/image"), 0u);
}

TEST_F(TestSubscriber, camera_sub_shutdown) {
  std::function<void(
      const sensor_msgs::msg::Image::ConstSharedPtr &,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &)> fcn =
    [](const auto & msg, const auto &) {(void)msg;};

  auto sub = image_transport::create_camera_subscription(node_.get(), "camera/image", fcn, "raw");
  EXPECT_EQ(node_->get_node_graph_interface()->count_subscribers("camera/image"), 1u);
  EXPECT_EQ(node_->get_node_graph_interface()->count_subscribers("camera/camera_info"), 1u);
  sub.shutdown();
  EXPECT_EQ(node_->get_node_graph_interface()->count_subscribers("camera/image"), 0u);
  EXPECT_EQ(node_->get_node_graph_interface()->count_subscribers("camera/camera_info"), 0u);
}

TEST_F(TestSubscriber, callback_groups) {
  using namespace std::chrono_literals;

  // Create a publisher node.
  auto node_publisher = rclcpp::Node::make_shared("image_publisher", rclcpp::NodeOptions());
  image_transport::ImageTransport it_publisher(node_publisher);
  image_transport::Publisher pub = it_publisher.advertise("camera/image", 1);

  auto msg = sensor_msgs::msg::Image();
  auto timer = node_publisher->create_wall_timer(100ms, [&]() {pub.publish(msg);});

  // Create a subscriber to read the images.
  std::atomic<bool> flag_1 = false;
  std::atomic<bool> flag_2 = false;
  std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr & msg)> fcn1 =
    [&](const auto & msg) {
      (void)msg;
      flag_1 = true;
      std::this_thread::sleep_for(1s);
    };
  std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr & msg)> fcn2 =
    [&](const auto & msg) {
      (void)msg;
      flag_2 = true;
      std::this_thread::sleep_for(1s);
    };

  auto cb_group = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cb_group;

  image_transport::ImageTransport it(node_);

  auto subscriber_1 = it.subscribe("camera/image", 1, fcn1, nullptr, nullptr, sub_options);
  auto subscriber_2 = it.subscribe("camera/image", 1, fcn2, nullptr, nullptr, sub_options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_);
  executor.add_node(node_publisher);
  // Both callbacks should be executed and the flags should be set.
  std::thread executor_thread([&]() {executor.spin();});

  // The callbacks sleep for 5 seconds and mutually exclusive callbacks should be blocked.
  // However, because of the the multithreaded executor and renentrant callback group,
  // the flags should be set, as the callbacks should be in different threads.
  auto timeout_elapsed = 0.0s;
  auto sleep_duration = 0.1s;
  auto timeout = 0.5s;

  while (!(flag_1 && flag_2)) {
    std::this_thread::sleep_for(sleep_duration);
    timeout_elapsed += sleep_duration;
  }
  executor.cancel();
  executor_thread.join();

  EXPECT_LT(timeout_elapsed, timeout);
}

TEST_F(TestSubscriber, callback_groups_custom_qos) {
  using namespace std::chrono_literals;

  // Create a publisher node.
  auto node_publisher = rclcpp::Node::make_shared("image_publisher", rclcpp::NodeOptions());
  image_transport::ImageTransport it_publisher(node_publisher);
  image_transport::Publisher pub = it_publisher.advertise(
    "camera/image",
    rmw_qos_profile_sensor_data);

  auto msg = sensor_msgs::msg::Image();
  auto timer = node_publisher->create_wall_timer(100ms, [&]() {pub.publish(msg);});

  // Create a subscriber to read the images.
  std::atomic<bool> flag_1 = false;
  std::atomic<bool> flag_2 = false;
  std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr & msg)> fcn1 =
    [&](const auto & msg) {
      (void)msg;
      flag_1 = true;
      std::this_thread::sleep_for(1s);
    };
  std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr & msg)> fcn2 =
    [&](const auto & msg) {
      (void)msg;
      flag_2 = true;
      std::this_thread::sleep_for(1s);
    };

  auto cb_group = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cb_group;

  image_transport::ImageTransport it(node_);

  auto subscriber_1 = it.subscribe(
    "camera/image", rmw_qos_profile_sensor_data, fcn1, nullptr,
    nullptr, sub_options);
  auto subscriber_2 = it.subscribe(
    "camera/image", rmw_qos_profile_sensor_data, fcn2, nullptr,
    nullptr, sub_options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_);
  executor.add_node(node_publisher);
  // Both callbacks should be executed and the flags should be set.
  std::thread executor_thread([&]() {executor.spin();});

  // The callbacks sleep for 5 seconds and mutually exclusive callbacks should be blocked.
  // However, because of the the multithreaded executor and renentrant callback group,
  // the flags should be set, as the callbacks should be in different threads.
  auto timeout_elapsed = 0.0s;
  auto sleep_duration = 0.1s;
  auto timeout = 0.5s;

  while (!(flag_1 && flag_2)) {
    std::this_thread::sleep_for(sleep_duration);
    timeout_elapsed += sleep_duration;
  }
  executor.cancel();
  executor_thread.join();

  EXPECT_LT(timeout_elapsed, timeout);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}

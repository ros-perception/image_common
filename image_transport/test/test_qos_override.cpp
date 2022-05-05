// Copyright (c) 2022 Open Source Robotics Foundation, Inc.
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

class TestQosOverride : public ::testing::Test
{
protected:
  void SetUp()
  {
    pub_node_ = rclcpp::Node::make_shared("test_publisher");
    qos_override_pub_node_ = rclcpp::Node::make_shared(
      "test_qos_override_publisher", rclcpp::NodeOptions().parameter_overrides(
    {
      rclcpp::Parameter(
        "qos_overrides./camera/image.publisher.reliability", "best_effort"),
    }));
    sub_node_ = rclcpp::Node::make_shared("test_subscriber");
    qos_override_sub_node_ = rclcpp::Node::make_shared(
      "test_qos_override_subscriber", rclcpp::NodeOptions().parameter_overrides(
    {
      rclcpp::Parameter(
        "qos_overrides./camera/image.subscription.reliability", "best_effort"),
    }));
  }

  rclcpp::Node::SharedPtr pub_node_;
  rclcpp::Node::SharedPtr qos_override_pub_node_;
  rclcpp::Node::SharedPtr sub_node_;
  rclcpp::Node::SharedPtr qos_override_sub_node_;
};

TEST_F(TestQosOverride, qos_override_publisher_without_options) {
  auto pub =
    image_transport::create_publisher(pub_node_.get(), "camera/image", rmw_qos_profile_default);
  auto endpoint_info_vec = pub_node_->get_publishers_info_by_topic("camera/image");
  EXPECT_EQ(endpoint_info_vec[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  pub.shutdown();

  pub = image_transport::create_publisher(
    qos_override_pub_node_.get(), "camera/image", rmw_qos_profile_default);

  endpoint_info_vec = qos_override_pub_node_->get_publishers_info_by_topic("camera/image");
  EXPECT_EQ(
    endpoint_info_vec[0].qos_profile().reliability(),
    rclcpp::ReliabilityPolicy::Reliable);
  pub.shutdown();
}

TEST_F(TestQosOverride, qos_override_publisher_with_options) {
  rclcpp::PublisherOptions options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions(
  {
    rclcpp::QosPolicyKind::Depth,
    rclcpp::QosPolicyKind::Durability,
    rclcpp::QosPolicyKind::History,
    rclcpp::QosPolicyKind::Reliability,
  });

  auto pub = image_transport::create_publisher(
    pub_node_.get(), "camera/image", rmw_qos_profile_default, options);
  auto endpoint_info_vec = pub_node_->get_publishers_info_by_topic("camera/image");
  EXPECT_EQ(endpoint_info_vec[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  pub.shutdown();

  pub = image_transport::create_publisher(
    qos_override_pub_node_.get(), "camera/image", rmw_qos_profile_default, options);

  endpoint_info_vec = qos_override_pub_node_->get_publishers_info_by_topic("camera/image");
  EXPECT_EQ(
    endpoint_info_vec[0].qos_profile().reliability(),
    rclcpp::ReliabilityPolicy::BestEffort);
  pub.shutdown();
}

TEST_F(TestQosOverride, qos_override_subscriber_without_options) {
  std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr & msg)> fcn =
    [](const auto & msg) {(void)msg;};

  auto sub = image_transport::create_subscription(
    sub_node_.get(), "camera/image", fcn, "raw", rmw_qos_profile_default);
  auto endpoint_info_vec = sub_node_->get_subscriptions_info_by_topic("camera/image");
  EXPECT_EQ(endpoint_info_vec[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  sub.shutdown();

  sub = image_transport::create_subscription(
    qos_override_sub_node_.get(), "camera/image", fcn, "raw", rmw_qos_profile_default);

  endpoint_info_vec = qos_override_sub_node_->get_subscriptions_info_by_topic("camera/image");
  EXPECT_EQ(
    endpoint_info_vec[0].qos_profile().reliability(),
    rclcpp::ReliabilityPolicy::Reliable);
}

TEST_F(TestQosOverride, qos_override_subscriber_with_options) {
  std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr & msg)> fcn =
    [](const auto & msg) {(void)msg;};

  rclcpp::SubscriptionOptions options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions(
  {
    rclcpp::QosPolicyKind::Depth,
    rclcpp::QosPolicyKind::Durability,
    rclcpp::QosPolicyKind::History,
    rclcpp::QosPolicyKind::Reliability,
  });

  auto sub = image_transport::create_subscription(
    sub_node_.get(), "camera/image", fcn, "raw", rmw_qos_profile_default, options);
  auto endpoint_info_vec = sub_node_->get_subscriptions_info_by_topic("camera/image");
  EXPECT_EQ(endpoint_info_vec[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  sub.shutdown();

  sub = image_transport::create_subscription(
    qos_override_sub_node_.get(), "camera/image", fcn, "raw", rmw_qos_profile_default, options);

  endpoint_info_vec = qos_override_sub_node_->get_subscriptions_info_by_topic("camera/image");
  EXPECT_EQ(
    endpoint_info_vec[0].qos_profile().reliability(),
    rclcpp::ReliabilityPolicy::BestEffort);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}

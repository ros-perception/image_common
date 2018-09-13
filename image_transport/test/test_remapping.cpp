#include <gtest/gtest.h>

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "image_transport/image_transport.h"

class TestPublisher : public ::testing::Test
{
protected:
  void SetUp()
  {
    auto context = rclcpp::contexts::default_context::get_global_default_context();
    std::vector<std::string> arguments;
    arguments.push_back("old_topic:=new_topic");
    std::vector<rclcpp::Parameter> initial_parameters;

    node_ = rclcpp::Node::make_shared(
      "node_name",
      "namespace",
      context,
      arguments,
      initial_parameters
    );
  }

  rclcpp::Node::SharedPtr node_;
};

TEST_F(TestPublisher, Publisher) {
  // Subscribe
  bool received{false};
  // FIXME: this should be `new_topic`
  auto sub = image_transport::create_subscription(node_, "old_topic",
    [&received](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
      (void)msg;
      received = true;
    }, "raw");

  // Publish
  auto pub = image_transport::create_publisher(node_, "old_topic");
  auto image = std::make_shared<sensor_msgs::msg::Image>();
  pub.publish(image);

  // Spin
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.spin_node_some(node_);

  EXPECT_TRUE(received);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}

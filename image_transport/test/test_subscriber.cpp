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
    node_ = rclcpp::Node::make_shared("test_subscriber");
  }

  rclcpp::Node::SharedPtr node_;
};

TEST_F(TestPublisher, construction_and_destruction) {
  std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr & msg)> fcn =
    [](const auto & msg) {(void)msg;};

  auto sub = image_transport::create_subscription(node_.get(), "camera/image", fcn, "raw");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.spin_node_some(node_);
}

TEST_F(TestPublisher, shutdown) {
  std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr & msg)> fcn =
    [](const auto & msg) {(void)msg;};

  auto sub = image_transport::create_subscription(node_.get(), "camera/image", fcn, "raw");
  EXPECT_EQ(node_->get_node_graph_interface()->count_subscribers("camera/image"), 1u);
  sub.shutdown();
  EXPECT_EQ(node_->get_node_graph_interface()->count_subscribers("camera/image"), 0u);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}

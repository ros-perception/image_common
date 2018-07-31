#include <gtest/gtest.h>

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "image_transport/image_transport.h"

class TestPublisher : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    // node = std::make_shared<rclcpp::Node>("image_transport", "/ns");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
};

TEST_F(TestPublisher, construction_and_destruction) {
  node = std::make_shared<rclcpp::Node>("image_transport", "/ns");
  rclcpp::SubscriptionBase::SharedPtr sub_actual = nullptr;
  image_transport::ImageTransport it;

  std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr& msg)> fcn =
    [](const auto& msg) { (void)msg; };

  image_transport::Subscriber sub = it.subscribe(node, "camera/image", fcn);
  sub_actual = sub.get_sub();
  node.reset();
}

#include <gtest/gtest.h>

#include <functional>
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "image_transport/single_subscriber_publisher.hpp"

class TestPublisher : public ::testing::Test
{
protected:
  static constexpr const char* caller_id = "node";
  static constexpr const char* topic = "/topic";


  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("image_transport", "/ns");
  }

  void TearDown()
  {
    node.reset();
  }


  rclcpp::Node::SharedPtr node;
};

TEST_F(TestPublisher, construction_and_destruction) {
  auto get_num_subscribers = []() -> size_t { return 0; };
  auto publish_fn= [](const sensor_msgs::msg::Image& /*image*/){};

  image_transport::SingleSubscriberPublisher ssp(caller_id, topic,
      get_num_subscribers, publish_fn);
}

TEST_F(TestPublisher, getNumSubscribers) {
  size_t nSub = 0;

  auto get_num_subscribers = [&nSub]() -> size_t { return nSub; };
  auto publish_fn= [](const sensor_msgs::msg::Image& /*image*/){};

  image_transport::SingleSubscriberPublisher ssp(caller_id, topic,
      get_num_subscribers, publish_fn);

  nSub = 0;
  ASSERT_EQ(ssp.getNumSubscribers(), 0u);
  nSub = 1;
  ASSERT_EQ(ssp.getNumSubscribers(), 1u);
}


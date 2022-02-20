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

TEST_F(TestPublisher, Publisher) {
  auto pub = image_transport::create_publisher(node_.get(), "camera/image");
}

TEST_F(TestPublisher, ImageTransportPublisher) {
  image_transport::ImageTransport it(node_);
  auto pub = it.advertise("camera/image", 1);
}

TEST_F(TestPublisher, CameraPublisher) {
  auto camera_pub = image_transport::create_camera_publisher(node_.get(), "camera/image");
}

TEST_F(TestPublisher, ImageTransportCameraPublisher) {
  image_transport::ImageTransport it(node_);
  auto pub = it.advertiseCamera("camera/image", 1);
}

TEST_F(TestPublisher, Shutdown) {
  auto pub = image_transport::create_publisher(node_.get(), "camera/image");
  EXPECT_EQ(node_->get_node_graph_interface()->count_publishers("camera/image"), 1u);
  pub.shutdown();
  EXPECT_EQ(node_->get_node_graph_interface()->count_publishers("camera/image"), 0u);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}

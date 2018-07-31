/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018 Open Robotics
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"

#include <gtest/gtest.h>

#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.h"

#include "utils.hpp"

using namespace std::chrono_literals;

int total_images_received = 0;

class MessagePassingTesting : public ::testing::Test
{
public:
  image_transport::ImageTransport it()
  {
    return image_transport::ImageTransport();
  }

  sensor_msgs::msg::Image::UniquePtr generate_random_image()
  {
    auto image = std::make_unique<sensor_msgs::msg::Image>();
    return image;
  }

protected:
  void SetUp()
  {
    node_ = rclcpp::Node::make_shared("test_message_passing");
    total_images_received = 0;
  }

  void TearDown() {
    std::cout << "TearDown" << std::endl;
    if(node_) {
      std::cout << "Node still valid" << std::endl;
    }
  }

  rclcpp::Node::SharedPtr node_;
};


void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  (void) msg;
  std::cout << "imageCallback" << std::endl;
  total_images_received++;
}

TEST_F(MessagePassingTesting, one_message_passing)
{
  const size_t max_retries = 3;
  const size_t max_loops = 200;
  const std::chrono::milliseconds sleep_per_loop = std::chrono::milliseconds(10);

  rclcpp::executors::SingleThreadedExecutor executor;

  image_transport::Publisher pub = it().advertise(node_, "camera/image");
  image_transport::Subscriber sub = it().subscribe(node_, "camera/image", imageCallback);

  test_rclcpp::wait_for_subscriber(node_, sub.getTopic());

  ASSERT_EQ(0, total_images_received);
  ASSERT_EQ(1u, pub.getNumSubscribers());
  ASSERT_EQ(1u, sub.getNumPublishers());

  executor.spin_node_some(node_);
  ASSERT_EQ(0, total_images_received);

  size_t retry = 0;
  while(retry < max_retries && total_images_received == 0) {
    // generate random image and publish it
    pub.publish(generate_random_image());

    executor.spin_node_some(node_);
    size_t loop = 0;
    while ((total_images_received != 1) && (loop++ < max_loops)) {
      std::this_thread::sleep_for(sleep_per_loop);
      executor.spin_node_some(node_);
    }
  }

  ASSERT_EQ(1, total_images_received);
}

/*
TEST_F(MessagePassingTesting, stress_message_passing)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  int images_to_stress = 1000;

  image_transport::Publisher pub = it().advertise("camera/image");
  image_transport::Subscriber sub = it().subscribe("camera/image", imageCallback);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // generate random image and publish it
  int image_pubs = 0;
  while (image_pubs < images_to_stress) {
    pub.publish(generate_random_image());
    executor.spin_some();
    image_pubs++;
  }

  ASSERT_EQ(images_to_stress, total_images_received);
}
*/

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  std::cout << "rclcpp::shutdown" << std::endl;
  rclcpp::shutdown();
  return ret;
}

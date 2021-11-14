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

#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "utils.hpp"

using namespace std::chrono_literals;

class IPCTesting : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options{};
    options.use_intra_process_comms(true);
    node_ = rclcpp::Node::make_shared("ipc_testing", options);
  }

  rclcpp::Node::SharedPtr node_;
  int count_ = 0;
  std::uintptr_t recv_image_addr_ = 0;
  std::uintptr_t recv_info_addr_ = 0;
};

TEST_F(IPCTesting, camera_ipc)
{
  const size_t max_retries = 3;
  const size_t max_loops = 200;
  const std::chrono::milliseconds sleep_per_loop = std::chrono::milliseconds(10);

  rclcpp::executors::SingleThreadedExecutor executor;

  auto pub = image_transport::create_camera_publisher(node_.get(), "camera/image");
  auto sub = image_transport::create_camera_subscription(node_.get(), "camera/image",
    [this](const sensor_msgs::msg::Image::ConstSharedPtr& image,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info)
      {
        count_++;
        recv_image_addr_ = reinterpret_cast<std::uintptr_t>(image.get());
        recv_info_addr_ = reinterpret_cast<std::uintptr_t>(info.get());
      },
      "raw"
    );

  test_rclcpp::wait_for_subscriber(node_, sub.getTopic());
  ASSERT_EQ(0, count_);

  executor.spin_node_some(node_);
  ASSERT_EQ(0, count_);

  size_t retry = 0;
  std::uintptr_t send_image_addr;
  std::uintptr_t send_info_addr;
  while(retry < max_retries && count_ == 0) {
    auto image = std::make_unique<sensor_msgs::msg::Image>();
    auto info = std::make_unique<sensor_msgs::msg::CameraInfo>();
    send_image_addr = reinterpret_cast<std::uintptr_t>(image.get());
    send_info_addr = reinterpret_cast<std::uintptr_t>(info.get());
    pub.publishUnique(std::move(image), std::move(info));

    executor.spin_node_some(node_);
    size_t loop = 0;
    while ((count_ != 1) && (loop++ < max_loops)) {
      std::this_thread::sleep_for(sleep_per_loop);
      executor.spin_node_some(node_);
    }
  }

  EXPECT_EQ(1, count_);
  EXPECT_EQ(send_image_addr, recv_image_addr_);
  EXPECT_EQ(send_info_addr, recv_info_addr_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}

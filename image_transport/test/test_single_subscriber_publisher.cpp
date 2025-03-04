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

#include <functional>
#include <string>
#include <memory>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "image_transport/single_subscriber_publisher.hpp"

class TestPublisher : public ::testing::Test
{
protected:
  static constexpr const char * caller_id = "node";
  static constexpr const char * topic = "/topic";


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

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node;
};

TEST_F(TestPublisher, construction_and_destruction) {
  auto get_num_subscribers = []() -> size_t {return 0;};
  auto publish_fn = [](const sensor_msgs::msg::Image & /*image*/) {};

  image_transport::SingleSubscriberPublisher ssp(caller_id, topic,
    get_num_subscribers, publish_fn);
}

TEST_F(TestPublisher, getNumSubscribers) {
  size_t nSub = 0;

  auto get_num_subscribers = [&nSub]() -> size_t {return nSub;};
  auto publish_fn = [](const sensor_msgs::msg::Image & /*image*/) {};

  image_transport::SingleSubscriberPublisher ssp(caller_id, topic,
    get_num_subscribers, publish_fn);

  nSub = 0;
  ASSERT_EQ(ssp.getNumSubscribers(), 0u);
  nSub = 1;
  ASSERT_EQ(ssp.getNumSubscribers(), 1u);
}

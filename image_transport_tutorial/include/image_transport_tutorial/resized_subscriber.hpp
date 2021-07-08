// Copyright 2021, Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef IMAGE_TRANSPORT_TUTORIAL__RESIZED_SUBSCRIBER_HPP_
#define IMAGE_TRANSPORT_TUTORIAL__RESIZED_SUBSCRIBER_HPP_

#include <image_transport/simple_subscriber_plugin.hpp>
#include <image_transport_tutorial/msg/resized_image.hpp>

#include <string>

class ResizedSubscriber : public image_transport::SimpleSubscriberPlugin
  <image_transport_tutorial::msg::ResizedImage>
{
public:
  virtual ~ResizedSubscriber() {}

  virtual std::string getTransportName() const
  {
    return "resized";
  }

protected:
  void subscribeImpl(
    rclcpp::Node * node,
    const std::string & base_topic,
    const Callback & callback,
    rmw_qos_profile_t custom_qos,
    rclcpp::SubscriptionOptions options) override
  {
    this->subscribeImplWithOptions(node, base_topic, callback, custom_qos, options);
  }

  virtual void internalCallback(
    const typename image_transport_tutorial::msg::ResizedImage::ConstSharedPtr & message,
    const Callback & user_cb);
};

#endif  // IMAGE_TRANSPORT_TUTORIAL__RESIZED_SUBSCRIBER_HPP_

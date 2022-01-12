// Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef IMAGE_TRANSPORT__RAW_SUBSCRIBER_HPP_
#define IMAGE_TRANSPORT__RAW_SUBSCRIBER_HPP_

#include <string>
#include <memory>

#include "sensor_msgs/msg/image.hpp"

#include "image_transport/simple_subscriber_plugin.hpp"
#include "image_transport/visibility_control.hpp"

namespace image_transport
{

/**
 * \brief The default SubscriberPlugin.
 *
 * RawSubscriber is a simple wrapper for ros::Subscriber which listens for Image messages
 * and passes them through to the callback.
 */
class RawSubscriber : public SimpleSubscriberPlugin<sensor_msgs::msg::Image>
{
public:
  virtual ~RawSubscriber() {}

  std::string getTransportName() const override
  {
    return "raw";
  }

protected:
  void internalCallback(
    const std::shared_ptr<const sensor_msgs::msg::Image> & message,
    const Callback & user_cb) override
  {
    user_cb(message);
  }

  std::string getTopicToSubscribe(const std::string & base_topic) const override
  {
    return base_topic;
  }

  using SubscriberPlugin::subscribeImpl;

  void subscribeImpl(
    rclcpp::Node * node,
    const std::string & base_topic,
    const Callback & callback,
    rmw_qos_profile_t custom_qos,
    rclcpp::SubscriptionOptions options) override
  {
    this->subscribeImplWithOptions(node, base_topic, callback, custom_qos, options);
  }
};

}  // namespace image_transport

#endif  // IMAGE_TRANSPORT__RAW_SUBSCRIBER_HPP_

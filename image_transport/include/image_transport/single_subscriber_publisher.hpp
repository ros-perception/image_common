/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef IMAGE_TRANSPORT__SINGLE_SUBSCRIBER_PUBLISHER_HPP_
#define IMAGE_TRANSPORT__SINGLE_SUBSCRIBER_PUBLISHER_HPP_

#include <rclcpp/macros.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <functional>
#include <string>

#include "image_transport/visibility_control.hpp"

namespace image_transport {

/**
 * \brief Allows publication of an image to a single subscriber. Only available inside
 * subscriber connection callbacks.
 */
class SingleSubscriberPublisher
{
private:
  SingleSubscriberPublisher(const SingleSubscriberPublisher&) = delete;
  SingleSubscriberPublisher& operator=( const SingleSubscriberPublisher& ) = delete;

public:
  typedef std::function<size_t()> GetNumSubscribersFn;
  typedef std::function<void(const sensor_msgs::msg::Image&)> PublishFn;

  IMAGE_TRANSPORT_PUBLIC
  SingleSubscriberPublisher(
    const std::string & caller_id, const std::string & topic,
    const GetNumSubscribersFn & num_subscribers_fn,
    const PublishFn & publish_fn);

  IMAGE_TRANSPORT_PUBLIC
  std::string getSubscriberName() const;

  IMAGE_TRANSPORT_PUBLIC
  std::string getTopic() const;

  IMAGE_TRANSPORT_PUBLIC
  size_t getNumSubscribers() const;

  IMAGE_TRANSPORT_PUBLIC
  void publish(const sensor_msgs::msg::Image& message) const;

  IMAGE_TRANSPORT_PUBLIC
  void publish(const sensor_msgs::msg::Image::ConstSharedPtr& message) const;

private:
  std::string caller_id_;
  std::string topic_;
  GetNumSubscribersFn num_subscribers_fn_;
  PublishFn publish_fn_;

  friend class Publisher; // to get publish_fn_ directly
};

typedef std::function<void (const SingleSubscriberPublisher &)> SubscriberStatusCallback;

}  // namespace image_transport

#endif  // IMAGE_TRANSPORT__SINGLE_SUBSCRIBER_PUBLISHER_HPP_

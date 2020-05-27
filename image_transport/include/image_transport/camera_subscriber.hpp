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

#ifndef IMAGE_TRANSPORT__CAMERA_SUBSCRIBER_HPP_
#define IMAGE_TRANSPORT__CAMERA_SUBSCRIBER_HPP_

#include <functional>

#include <rclcpp/node.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "image_transport/visibility_control.hpp"

namespace image_transport {

class ImageTransport;

/**
 * \brief Manages a subscription callback on synchronized Image and CameraInfo topics.
 *
 * CameraSubscriber is the client-side counterpart to CameraPublisher, and assumes the
 * same topic naming convention. The callback is of type:
\verbatim
void callback(const sensor_msgs::msg::Image::ConstSharedPtr&, const sensor_msgs::msg::CameraInfo::ConstSharedPtr&);
\endverbatim
 *
 * A CameraSubscriber should always be created through a call to
 * ImageTransport::subscribeCamera(), or copied from one that was.
 * Once all copies of a specific CameraSubscriber go out of scope, the subscription callback
 * associated with that handle will stop being called. Once all CameraSubscriber for a given
 * topic go out of scope the topic will be unsubscribed.
 */
class CameraSubscriber
{
public:
  typedef std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr&,
                             const sensor_msgs::msg::CameraInfo::ConstSharedPtr&)> Callback;

  IMAGE_TRANSPORT_PUBLIC
  CameraSubscriber() = default;

  IMAGE_TRANSPORT_PUBLIC
  CameraSubscriber(rclcpp::Node * node,
                   const std::string& base_topic,
                   const Callback& callback,
                   const std::string& transport,
                   rmw_qos_profile_t = rmw_qos_profile_default);

  /**
   * \brief Get the base topic (on which the raw image is published).
   */
  IMAGE_TRANSPORT_PUBLIC
  std::string getTopic() const;

  /**
   * \brief Get the camera info topic.
   */
  IMAGE_TRANSPORT_PUBLIC
  std::string getInfoTopic() const;

  /**
   * \brief Returns the number of publishers this subscriber is connected to.
   */
  IMAGE_TRANSPORT_PUBLIC
  size_t getNumPublishers() const;

  /**
   * \brief Returns the name of the transport being used.
   */
  IMAGE_TRANSPORT_PUBLIC
  std::string getTransport() const;

  /**
   * \brief Unsubscribe the callback associated with this CameraSubscriber.
   */
  IMAGE_TRANSPORT_PUBLIC
  void shutdown();

  IMAGE_TRANSPORT_PUBLIC
  operator void*() const;

  IMAGE_TRANSPORT_PUBLIC
  bool operator< (const CameraSubscriber& rhs) const { return impl_ <  rhs.impl_; }

  IMAGE_TRANSPORT_PUBLIC
  bool operator!=(const CameraSubscriber& rhs) const { return impl_ != rhs.impl_; }

  IMAGE_TRANSPORT_PUBLIC
  bool operator==(const CameraSubscriber& rhs) const { return impl_ == rhs.impl_; }

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace image_transport

#endif

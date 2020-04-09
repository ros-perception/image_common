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

#ifndef IMAGE_TRANSPORT__CAMERA_PUBLISHER_HPP_
#define IMAGE_TRANSPORT__CAMERA_PUBLISHER_HPP_

#include <rclcpp/macros.hpp>
#include <rclcpp/node.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "image_transport/single_subscriber_publisher.hpp"
#include "image_transport/visibility_control.hpp"

namespace image_transport
{

class ImageTransport;

/**
 * \brief Manages advertisements for publishing camera images.
 *
 * CameraPublisher is a convenience class for publishing synchronized image and
 * camera info topics using the standard topic naming convention, where the info
 * topic name is "camera_info" in the same namespace as the base image topic.
 *
 * On the client side, CameraSubscriber simplifies subscribing to camera images.
 *
 * A CameraPublisher should always be created through a call to
 * ImageTransport::advertiseCamera(), or copied from one that was.
 * Once all copies of a specific CameraPublisher go out of scope, any subscriber callbacks
 * associated with that handle will stop being called. Once all CameraPublisher for a
 * given base topic go out of scope the topic (and all subtopics) will be unadvertised.
 */
class CameraPublisher
{
public:
  IMAGE_TRANSPORT_PUBLIC
  CameraPublisher() = default;

  IMAGE_TRANSPORT_PUBLIC
  CameraPublisher(
    rclcpp::Node * node,
    const std::string & base_topic,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default);

  //TODO(ros2) Restore support for SubscriberStatusCallbacks when available.

  /*!
   * \brief Returns the number of subscribers that are currently connected to
   * this CameraPublisher.
   *
   * Returns max(image topic subscribers, info topic subscribers).
   */
  IMAGE_TRANSPORT_PUBLIC
  size_t getNumSubscribers() const;

  /*!
   * \brief Returns the base (image) topic of this CameraPublisher.
   */
  IMAGE_TRANSPORT_PUBLIC
  std::string getTopic() const;

  /**
   * \brief Returns the camera info topic of this CameraPublisher.
   */
  IMAGE_TRANSPORT_PUBLIC
  std::string getInfoTopic() const;

  /*!
   * \brief Publish an (image, info) pair on the topics associated with this CameraPublisher.
   */
  IMAGE_TRANSPORT_PUBLIC
  void publish(
    const sensor_msgs::msg::Image & image,
    const sensor_msgs::msg::CameraInfo & info) const;

  /*!
   * \brief Publish an (image, info) pair on the topics associated with this CameraPublisher.
   */
  IMAGE_TRANSPORT_PUBLIC
  void publish(
    const sensor_msgs::msg::Image::ConstSharedPtr & image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info) const;

  /*!
   * \brief Publish an (image, info) pair with given timestamp on the topics associated with
   * this CameraPublisher.
   *
   * Convenience version, which sets the timestamps of both image and info to stamp before
   * publishing.
   */
  IMAGE_TRANSPORT_PUBLIC
  void publish(
    sensor_msgs::msg::Image & image, sensor_msgs::msg::CameraInfo & info,
    rclcpp::Time stamp) const;

  /*!
   * \brief Shutdown the advertisements associated with this Publisher.
   */
  IMAGE_TRANSPORT_PUBLIC
  void shutdown();

  IMAGE_TRANSPORT_PUBLIC
  operator void *() const;

  IMAGE_TRANSPORT_PUBLIC
  bool operator<(const CameraPublisher & rhs) const {return impl_ < rhs.impl_;}

  IMAGE_TRANSPORT_PUBLIC
  bool operator!=(const CameraPublisher & rhs) const {return impl_ != rhs.impl_;}

  IMAGE_TRANSPORT_PUBLIC
  bool operator==(const CameraPublisher & rhs) const {return impl_ == rhs.impl_;}

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace image_transport

#endif  // IMAGE_TRANSPORT__CAMERA_PUBLISHER_HPP_

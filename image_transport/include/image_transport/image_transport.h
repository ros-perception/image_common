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

#ifndef IMAGE_TRANSPORT_IMAGE_TRANSPORT_H
#define IMAGE_TRANSPORT_IMAGE_TRANSPORT_H

#include "image_transport/publisher.h"
#include "image_transport/subscriber.h"
#include "image_transport/camera_publisher.h"
#include "image_transport/camera_subscriber.h"

namespace image_transport
{

/**
 * \brief Advertise and subscribe to image topics.
 *
 * ImageTransport is analogous to ros::NodeHandle in that it contains advertise() and
 * subscribe() functions for creating advertisements and subscriptions of image topics.
 */
class ImageTransport
{
public:
  ImageTransport();

  ~ImageTransport();

  /*!
   * \brief Advertise an image topic, simple version.
   */
  Publisher advertise(
    rclcpp::Node::SharedPtr node,
    const std::string & base_topic,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default);

  /**
   * \brief Subscribe to an image topic, version for arbitrary std::function object.
   */
  Subscriber subscribe(
    rclcpp::Node::SharedPtr node,
    const std::string & base_topic,
    const std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr &)> & callback,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default);

  /**
   * \brief Subscribe to an image topic, version for bare function.
   */
  Subscriber subscribe(
    rclcpp::Node::SharedPtr node,
    const std::string & base_topic,
    void (*fp)(const sensor_msgs::msg::Image::ConstSharedPtr &),
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default)
  {
    return subscribe(node, base_topic,
             std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr &)>(fp), custom_qos);
  }

  /**
   * \brief Subscribe to an image topic, version for class member function with bare pointer.
   */
  template<class T>
  Subscriber subscribe(
    rclcpp::Node::SharedPtr node,
    const std::string & base_topic,
    void (T::* fp)(const sensor_msgs::msg::Image::ConstSharedPtr &), T * obj,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default)
  {
    return subscribe(node, base_topic,
             std::bind(fp, obj, std::placeholders::_1), custom_qos);
  }

  /*!
   * \brief Advertise a synchronized camera raw image + info topic pair, simple version.
   */
  CameraPublisher advertiseCamera(
    rclcpp::Node::SharedPtr node,
    const std::string & base_topic,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default);

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for arbitrary
   * std::function object.
   *
   * This version assumes the standard topic naming scheme, where the info topic is
   * named "camera_info" in the same namespace as the base image topic.
   */
  CameraSubscriber subscribeCamera(
    rclcpp::Node::SharedPtr node,
    const std::string & base_topic,
    const CameraSubscriber::Callback & callback,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default);

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for bare function.
   */
  CameraSubscriber subscribeCamera(
    rclcpp::Node::SharedPtr node,
    const std::string & base_topic,
    void (*fp)(const sensor_msgs::msg::Image::ConstSharedPtr &,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &),
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default)
  {
    return subscribeCamera(node, base_topic, CameraSubscriber::Callback(fp), custom_qos);
  }

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for class member
   * function with bare pointer.
   */
  template<class T>
  CameraSubscriber subscribeCamera(
    rclcpp::Node::SharedPtr node,
    const std::string & base_topic,
    void (T::* fp)(const sensor_msgs::msg::Image::ConstSharedPtr &,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &), T * obj,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default)
  {
    return subscribeCamera(node, base_topic,
             std::bind(fp, obj, std::placeholders::_1, std::placeholders::_2), custom_qos);
  }

  /**
   * \brief Returns the names of all transports declared in the system. Declared
   * transports are not necessarily built or loadable.
   */
  std::vector<std::string> getDeclaredTransports() const;

  /**
   * \brief Returns the names of all transports that are loadable in the system.
   */
  std::vector<std::string> getLoadableTransports() const;

private:
  struct Impl;
  typedef std::shared_ptr<Impl> ImplPtr;
  typedef std::weak_ptr<Impl> ImplWPtr;

  ImplPtr impl_;
};

} //namespace image_transport

#endif

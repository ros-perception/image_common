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

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>

#include "image_transport/camera_publisher.h"
#include "image_transport/camera_subscriber.h"
#include "image_transport/publisher.h"
#include "image_transport/subscriber.h"
#include "image_transport/transport_hints.h"

namespace image_transport
{

/*!
 * \brief Advertise an image topic, free function version.
 */
Publisher create_publisher(
  rclcpp::Node::SharedPtr node,
  const std::string & base_topic,
  rmw_qos_profile_t custom_qos = rmw_qos_profile_default);

/**
 * \brief Subscribe to an image topic, free function version.
 */
Subscriber create_subscription(
  rclcpp::Node::SharedPtr node,
  const std::string & base_topic,
  const Subscriber::Callback & callback,
  const std::string & transport,
  rmw_qos_profile_t custom_qos = rmw_qos_profile_default);

/*!
 * \brief Advertise a camera, free function version.
 */
CameraPublisher create_camera_publisher(
  rclcpp::Node::SharedPtr node,
  const std::string & base_topic,
  rmw_qos_profile_t custom_qos = rmw_qos_profile_default);

/*!
 * \brief Subscribe to a camera, free function version.
 */
CameraSubscriber create_camera_subscription(
  rclcpp::Node::SharedPtr node,
  const std::string & base_topic,
  const CameraSubscriber::Callback & callback,
  const std::string & transport,
  rmw_qos_profile_t custom_qos = rmw_qos_profile_default);

std::vector<std::string> getDeclaredTransports();
std::vector<std::string> getLoadableTransports();

/**
 * \brief Advertise and subscribe to image topics.
 *
 * ImageTransport is analogous to ros::NodeHandle in that it contains advertise() and
 * subscribe() functions for creating advertisements and subscriptions of image topics.
*/
class ImageTransport
{
public:
  using VoidPtr = std::shared_ptr<void>;
  using ImageConstPtr = sensor_msgs::msg::Image::ConstSharedPtr;
  using CameraInfoConstPtr = sensor_msgs::msg::CameraInfo::ConstSharedPtr;

  explicit ImageTransport(rclcpp::Node::SharedPtr node);

  ~ImageTransport();

  /*!
   * \brief Advertise an image topic, simple version.
   */
  Publisher advertise(const std::string & base_topic, uint32_t queue_size, bool latch = false);

  /*!
   * \brief Advertise an image topic with subcriber status callbacks.
   */
  /* TODO(ros2) Implement when SubscriberStatusCallback is available
   * Publisher advertise(const std::string& base_topic, uint32_t queue_size,
   *                    const SubscriberStatusCallback& connect_cb,
   *                    const SubscriberStatusCallback& disconnect_cb = SubscriberStatusCallback(),
   *                    const ros::VoidPtr& tracked_object = ros::VoidPtr(), bool latch = false);
   */

  /**
   * \brief Subscribe to an image topic, version for arbitrary std::function object.
   */
  Subscriber subscribe(
    const std::string & base_topic, uint32_t queue_size,
    const Subscriber::Callback & callback,
    const VoidPtr & tracked_object = VoidPtr(),
    const TransportHints * transport_hints = nullptr);

  /**
   * \brief Subscribe to an image topic, version for bare function.
   */
  Subscriber subscribe(
    const std::string & base_topic, uint32_t queue_size,
    void (*fp)(const ImageConstPtr &),
    const TransportHints * transport_hints = nullptr)
  {
    return subscribe(base_topic, queue_size,
             std::function<void(const ImageConstPtr &)>(fp),
             VoidPtr(), transport_hints);
  }

  /**
   * \brief Subscribe to an image topic, version for class member function with bare pointer.
   */
  template<class T>
  Subscriber subscribe(
    const std::string & base_topic, uint32_t queue_size,
    void (T::* fp)(const ImageConstPtr &), T * obj,
    const TransportHints * transport_hints = nullptr)
  {
    return subscribe(base_topic, queue_size, std::bind(fp, obj, std::placeholders::_1),
             VoidPtr(), transport_hints);
  }

  /**
   * \brief Subscribe to an image topic, version for class member function with shared_ptr.
   */
  template<class T>
  Subscriber subscribe(
    const std::string & base_topic, uint32_t queue_size,
    void (T::* fp)(const ImageConstPtr &),
    const std::shared_ptr<T> & obj,
    const TransportHints * transport_hints = nullptr)
  {
    return subscribe(base_topic, queue_size, std::bind(fp,
             obj.get(), std::placeholders::_1), obj, transport_hints);
  }

  /*!
   * \brief Advertise a synchronized camera raw image + info topic pair, simple version.
   */
  CameraPublisher advertiseCamera(
    const std::string & base_topic, uint32_t queue_size,
    bool latch = false);

  /*!
   * \brief Advertise a synchronized camera raw image + info topic pair with subscriber status
   * callbacks.
   */
  /* TODO(ros2) Implement when SubscriberStatusCallback is available
   * CameraPublisher advertiseCamera(const std::string& base_topic, uint32_t queue_size,
   *                                const SubscriberStatusCallback& image_connect_cb,
   *                                const SubscriberStatusCallback& image_disconnect_cb = SubscriberStatusCallback(),
   *                                const ros::SubscriberStatusCallback& info_connect_cb = ros::SubscriberStatusCallback(),
   *                                const ros::SubscriberStatusCallback& info_disconnect_cb = ros::SubscriberStatusCallback(),
   *                                const ros::VoidPtr& tracked_object = ros::VoidPtr(), bool latch = false);
   */

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for arbitrary
   * std::function object.
   *
   * This version assumes the standard topic naming scheme, where the info topic is
   * named "camera_info" in the same namespace as the base image topic.
   */
  CameraSubscriber subscribeCamera(
    const std::string & base_topic, uint32_t queue_size,
    const CameraSubscriber::Callback & callback,
    const VoidPtr & tracked_object = VoidPtr(),
    const TransportHints * transport_hints = nullptr);

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for bare function.
   */
  CameraSubscriber subscribeCamera(
    const std::string & base_topic, uint32_t queue_size,
    void (*fp)(const ImageConstPtr &,
               const CameraInfoConstPtr &),
    const TransportHints * transport_hints = nullptr)
  {
    return subscribeCamera(base_topic, queue_size, CameraSubscriber::Callback(fp), VoidPtr(),
             transport_hints);
  }

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for class member
   * function with bare pointer.
   */
  template<class T>
  CameraSubscriber subscribeCamera(
    const std::string & base_topic, uint32_t queue_size,
    void (T::* fp)(const ImageConstPtr &,
                   const CameraInfoConstPtr &), T * obj,
    const TransportHints * transport_hints = nullptr)
  {
    return subscribeCamera(base_topic, queue_size,
             std::bind(fp, obj, std::placeholders::_1, std::placeholders::_2), VoidPtr(),
             transport_hints);
  }

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for class member
   * function with shared_ptr.
   */
  template<class T>
  CameraSubscriber subscribeCamera(
    const std::string & base_topic, uint32_t queue_size,
    void (T::* fp)(const ImageConstPtr &,
                   const CameraInfoConstPtr &),
    const std::shared_ptr<T> & obj,
    const TransportHints * transport_hints = nullptr)
  {
    return subscribeCamera(base_topic, queue_size,
             std::bind(fp, obj.get(), std::placeholders::_1, std::placeholders::_2), obj,
             transport_hints);
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

  std::string getTransportOrDefault(const TransportHints * transport_hints);


  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} //namespace image_transport

#endif

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

#include "image_transport/publisher.h"
#include "image_transport/subscriber.h"
#include "image_transport/camera_publisher.h"
#include "image_transport/camera_subscriber.h"

namespace image_transport {

Publisher create_publisher(
    rclcpp::Node::SharedPtr node,
    const std::string & base_topic,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default);

Subscriber create_subscription(
    rclcpp::Node::SharedPtr node,
    const std::string & base_topic,
    const Subscriber::Callback& callback,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default);

CameraPublisher create_camera_publisher(
    rclcpp::Node::SharedPtr node,
    const std::string & base_topic,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default);

CameraSubscriber create_camera_subscription(
    rclcpp::Node::SharedPtr node,
    const std::string & base_topic,
    const CameraSubscriber::Callback & callback,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default);

std::vector<std::string> getDeclaredTransports();
std::vector<std::string> getLoadableTransports();

class ImageTransport
{
public:
  ImageTransport(rclcpp::Node::SharedPtr node);
  ~ImageTransport();

  Publisher advertise(
    const std::string & base_topic,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default);

  Subscriber subscribe(
    const std::string & base_topic,
    const Subscriber::Callback& callback,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default);

  Subscriber subscribe(
    const std::string & base_topic,
    void (*fp)(const sensor_msgs::msg::Image::ConstSharedPtr&),
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default)
  {
    return subscribe(base_topic, Subscriber::Callback(fp), custom_qos);
  }

  template<class T>
  Subscriber subscribe(
    const std::string & base_topic,
    void (T::*fp)(const sensor_msgs::msg::Image::ConstSharedPtr&),
    T* obj,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default)
  {
    return subscribe(base_topic, std::bind(fp, obj, std::placeholders::_1), custom_qos);
  }

  template<class T>
  Subscriber subscribe(
    const std::string & base_topic,
    void (T::*fp)(const sensor_msgs::msg::Image::ConstSharedPtr&),
    const std::shared_ptr<T>& obj,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default)
  {
    return subscribe(base_topic, std::bind(fp, obj, std::placeholders::_1), custom_qos);
  }

  CameraPublisher advertise_camera(
    const std::string & base_topic,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default);

  CameraSubscriber subscribe_camera(
    const std::string & base_topic,
    const CameraSubscriber::Callback& callback,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default);

  CameraSubscriber subscribe_camera(
    const std::string & base_topic,
    void (*fp)(const sensor_msgs::msg::Image::ConstSharedPtr&,
               const sensor_msgs::msg::CameraInfo::ConstSharedPtr&),
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default)
  {
    return subscribe_camera(base_topic, CameraSubscriber::Callback(fp), custom_qos);
  }

  template<class T>
  CameraSubscriber subscribe_camera(
    const std::string & base_topic,
    void (T::*fp)(const sensor_msgs::msg::Image::ConstSharedPtr&,
                  const sensor_msgs::msg::CameraInfo::ConstSharedPtr&),
    T* obj,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default)
  {
    return subscribe_camera(base_topic, std::bind(fp, obj, std::placeholders::_1), custom_qos);
  }

  template<class T>
  CameraSubscriber subscribe_camera(
    const std::string & base_topic,
    void (T::*fp)(const sensor_msgs::msg::Image::ConstSharedPtr&,
                  const sensor_msgs::msg::CameraInfo::ConstSharedPtr&),
    const std::shared_ptr<T> obj,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default)
  {
    return subscribe_camera(base_topic, std::bind(fp, obj, std::placeholders::_1), custom_qos);
  }

  std::vector<std::string> getDeclaredTransports() const;
  std::vector<std::string> getLoadableTransports() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} //namespace image_transport

#endif

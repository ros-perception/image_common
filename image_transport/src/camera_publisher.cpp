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

#include "image_transport/image_transport.h"
#include "image_transport/camera_common.h"

namespace image_transport {

struct CameraPublisher::Impl
{
  Impl()
  {
  }

  ~Impl()
  {
  }
  
  void shutdown()
  {
    image_pub_.shutdown();
    info_pub_.shutdown();
  }

  Publisher image_pub_;
  ros::Publisher info_pub_;
};

CameraPublisher::CameraPublisher()
{
}

CameraPublisher::CameraPublisher(const CameraPublisher& rhs)
  : impl_(rhs.impl_)
{
}

CameraPublisher::~CameraPublisher()
{
}

CameraPublisher::CameraPublisher(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                                 const SubscriberStatusCallback& image_connect_cb,
                                 const SubscriberStatusCallback& image_disconnect_cb,
                                 const ros::SubscriberStatusCallback& info_connect_cb,
                                 const ros::SubscriberStatusCallback& info_disconnect_cb,
                                 const ros::VoidPtr& tracked_object, bool latch)
  : impl_(new Impl)
{
  std::string info_topic = getCameraInfoTopic(base_topic);

  /// @todo Why doesn't ImageTransport just pass in *this?
  ImageTransport it(nh);
  impl_->image_pub_ = it.advertise(base_topic, queue_size, image_connect_cb,
                                   image_disconnect_cb, tracked_object, latch);
  impl_->info_pub_ = nh.advertise<sensor_msgs::CameraInfo>(info_topic, queue_size, info_connect_cb,
                                                           info_disconnect_cb, tracked_object, latch);
}

uint32_t CameraPublisher::getNumSubscribers() const
{
  return std::max(impl_->image_pub_.getNumSubscribers(), impl_->info_pub_.getNumSubscribers());
}

std::string CameraPublisher::getTopic() const
{
  return impl_->image_pub_.getTopic();
}

std::string CameraPublisher::getInfoTopic() const
{
  return impl_->info_pub_.getTopic();
}

void CameraPublisher::publish(const sensor_msgs::Image& image, const sensor_msgs::CameraInfo& info) const
{
  impl_->image_pub_.publish(image);
  impl_->info_pub_.publish(info);
}

void CameraPublisher::publish(const sensor_msgs::ImageConstPtr& image,
                              const sensor_msgs::CameraInfoConstPtr& info) const
{
  impl_->image_pub_.publish(image);
  impl_->info_pub_.publish(info);
}

void CameraPublisher::publish(sensor_msgs::Image& image, sensor_msgs::CameraInfo& info,
                              ros::Time stamp) const
{
  image.header.stamp = stamp;
  info.header.stamp = stamp;
  publish(image, info);
}

void CameraPublisher::shutdown()
{
  impl_->shutdown();
}

} //namespace image_transport

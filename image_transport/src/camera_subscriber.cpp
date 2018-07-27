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

#include "image_transport/camera_subscriber.h"
#include "image_transport/subscriber_filter.h"
#include "image_transport/camera_common.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

inline void increment(int* value)
{
  ++(*value);
}

namespace image_transport {

struct CameraSubscriber::Impl
{
  Impl(rmw_qos_profile_t custom_qos)
    : //sync_(custom_qos),
      unsubscribed_(false),
      image_received_(0), info_received_(0), both_received_(0)
  {}

  ~Impl()
  {
    shutdown();
  }

  bool isValid() const
  {
    return !unsubscribed_;
  }

  void shutdown()
  {
    if (!unsubscribed_) {
      unsubscribed_ = true;
      //image_sub_.unsubscribe();
      //info_sub_.unsubscribe();
    }
  }

  void checkImagesSynchronized()
  {
    int threshold = 3 * both_received_;
    if (image_received_ > threshold || info_received_ > threshold) {
      /*
      ROS_WARN_NAMED("sync", // Can suppress ros.image_transport.sync independent of anything else
                     "[image_transport] Topics '%s' and '%s' do not appear to be synchronized. "
                     "In the last 10s:\n"
                     "\tImage messages received:      %d\n"
                     "\tCameraInfo messages received: %d\n"
                     "\tSynchronized pairs:           %d",
                     image_sub_.getTopic().c_str(), info_sub_.getTopic().c_str(),
                     image_received_, info_received_, both_received_);
      */
    }
    image_received_ = info_received_ = both_received_ = 0;
  }

  SubscriberFilter image_sub_;

  //message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
  //message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> sync_;
  bool unsubscribed_;
  // For detecting when the topics aren't synchronized
  //ros::WallTimer check_synced_timer_;
  int image_received_, info_received_, both_received_;
};

CameraSubscriber::CameraSubscriber(ImageTransport& image_it, rclcpp::Node::SharedPtr& node,
                                   const std::string& base_topic,
                                   const Callback& callback,
                                   rmw_qos_profile_t custom_qos)
  : impl_(new Impl(custom_qos))
{
  // Must explicitly remap the image topic since we then do some string manipulation on it
  // to figure out the sibling camera_info topic.
  //std::string image_topic = node.resolveName(base_topic);
  std::string image_topic = base_topic;
  std::string info_topic = getCameraInfoTopic(image_topic);

  //impl_->image_sub_.subscribe(image_it, image_topic, custom_qos);
  //impl_->info_sub_ .subscribe(info_nh, info_topic, custom_qos);

  //impl_->sync_.connectInput(impl_->image_sub_, impl_->info_sub_);
  // need for Boost.Bind here is kind of broken
  //impl_->sync_.registerCallback(std::bind(callback, std::placeholders::_1, std::placeholders::_2));

  // Complain every 10s if it appears that the image and info topics are not synchronized
  //impl_->image_sub_.registerCallback(std::bind(increment, &impl_->image_received_));
  //impl_->info_sub_.registerCallback(std::bind(increment, &impl_->info_received_));
  //impl_->sync_.registerCallback(std::bind(increment, &impl_->both_received_));
  //impl_->check_synced_timer_ = info_nh.createWallTimer(ros::WallDuration(10.0),
  //                                                    std::bind(&Impl::checkImagesSynchronized, impl_.get()));
}

std::string CameraSubscriber::getTopic() const
{
  //if (impl_) return impl_->image_sub_.getTopic();
  return std::string();
}

std::string CameraSubscriber::getInfoTopic() const
{
  //if (impl_) return impl_->info_sub_.getTopic();
  return std::string();
}

uint32_t CameraSubscriber::getNumPublishers() const
{
  /// @todo Fix this when message_filters::Subscriber has getNumPublishers()
  //if (impl_) return std::max(impl_->image_sub_.getNumPublishers(), impl_->info_sub_.getNumPublishers());
  //if (impl_) return impl_->image_sub_.getNumPublishers();
  return 0;
}

std::string CameraSubscriber::getTransport() const
{
  //if (impl_) return impl_->image_sub_.getTransport();
  return std::string();
}

void CameraSubscriber::shutdown()
{
  if (impl_) impl_->shutdown();
}

CameraSubscriber::operator void*() const
{
  return (impl_ && impl_->isValid()) ? (void*)1 : (void*)0;
}

} //namespace image_transport

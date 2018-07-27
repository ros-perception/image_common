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

#include "image_transport/camera_common.h"
#include "image_transport/image_transport.h"
#include "image_transport/publisher_plugin.h"
#include "image_transport/subscriber_plugin.h"

#include <pluginlib/class_loader.hpp>

namespace image_transport {

struct ImageTransport::Impl
{
  rclcpp::Node::SharedPtr node_;
  PubLoaderPtr pub_loader_;
  SubLoaderPtr sub_loader_;

  Impl(const rclcpp::Node::SharedPtr& node)
    : node_(node),
      pub_loader_( std::make_shared<PubLoader>("image_transport", "image_transport::PublisherPlugin") ),
      sub_loader_( std::make_shared<SubLoader>("image_transport", "image_transport::SubscriberPlugin") )
  {
  }
};

ImageTransport::ImageTransport(const rclcpp::Node::SharedPtr& node)
  : impl_(new Impl(node))
{
}

ImageTransport::~ImageTransport()
{
}

Publisher ImageTransport::advertise(const std::string& base_topic, uint32_t queue_size, bool latch)
{
  return Publisher(impl_->node_, base_topic, queue_size, latch, impl_->pub_loader_);
}

Subscriber ImageTransport::subscribe(const std::string& base_topic, uint32_t queue_size,
                                     const std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr&)>& callback)
{
  return Subscriber(impl_->node_, base_topic, queue_size, callback, impl_->sub_loader_);
}

CameraPublisher ImageTransport::advertiseCamera(const std::string& base_topic, uint32_t queue_size, bool latch)
{
  return advertiseCamera(base_topic, queue_size, latch);
}

CameraSubscriber ImageTransport::subscribeCamera(const std::string& base_topic, uint32_t queue_size,
                                                 const CameraSubscriber::Callback& callback)
{
  return CameraSubscriber(*this, impl_->node_, base_topic, queue_size, callback);
}

std::vector<std::string> ImageTransport::getDeclaredTransports() const
{
  std::vector<std::string> transports = impl_->sub_loader_->getDeclaredClasses();
  // Remove the "_sub" at the end of each class name.
  for(std::string& transport: transports) {
    transport = erase_last_copy(transport, "_sub");
  }
  return transports;
}

std::vector<std::string> ImageTransport::getLoadableTransports() const
{
  std::vector<std::string> loadableTransports;

  for( const std::string& transportPlugin: impl_->sub_loader_->getDeclaredClasses() )
  {
    // If the plugin loads without throwing an exception, add its
    // transport name to the list of valid plugins, otherwise ignore
    // it.
    try
    {
      std::shared_ptr<image_transport::SubscriberPlugin> sub = impl_->sub_loader_->createUniqueInstance(transportPlugin);
      loadableTransports.push_back(erase_last_copy(transportPlugin, "_sub")); // Remove the "_sub" at the end of each class name.
    }
    catch (const pluginlib::LibraryLoadException& e) {}
    catch (const pluginlib::CreateClassException& e) {}
  }

  return loadableTransports;

}

} //namespace image_transport

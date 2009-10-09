#include "image_transport/camera_subscriber.h"
#include "image_transport/subscriber_filter.h"
#include "image_transport/camera_common.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

namespace image_transport {

struct CameraSubscriber::Impl
{
  Impl(uint32_t queue_size)
    : sync_(queue_size)
  {}
  
  std::string info_topic_;
  SubscriberFilter image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> sync_;
};

CameraSubscriber::CameraSubscriber()
{
}

CameraSubscriber::~CameraSubscriber()
{
}

CameraSubscriber::CameraSubscriber(ros::NodeHandle& nh, const std::string& base_topic,
                                   uint32_t queue_size, const Callback& callback,
                                   const ros::VoidPtr& tracked_object,
                                   const TransportHints& transport_hints)
  : impl_(new Impl(queue_size))
{
  impl_->info_topic_ = getCameraInfoTopic(base_topic);
  impl_->image_sub_.subscribe(nh, base_topic, queue_size, transport_hints);
  impl_->info_sub_.subscribe(nh, impl_->info_topic_, queue_size, transport_hints.getRosHints());
  impl_->sync_.connectInput(impl_->image_sub_, impl_->info_sub_);
  // need for Boost.Bind here is kind of broken
  impl_->sync_.registerCallback(boost::bind(callback, _1, _2));
}

std::string CameraSubscriber::getTopic() const
{
  return impl_->image_sub_.getTopic();
}

std::string CameraSubscriber::getInfoTopic() const
{
  return impl_->info_topic_;
}

void CameraSubscriber::shutdown()
{
  impl_.reset();
}

} //namespace image_transport

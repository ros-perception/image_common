#include "image_transport/camera_subscriber.h"
#include "image_transport/subscriber_filter.h"
#include "image_transport/camera_common.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

namespace image_transport {

struct CameraSubscriber::Impl
{
  Impl(uint32_t queue_size)
    : sync_(queue_size),
      unsubscribed_(false)
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
      image_sub_.unsubscribe();
      info_sub_.unsubscribe();
    }
  }
  
  SubscriberFilter image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> sync_;
  bool unsubscribed_;
};

CameraSubscriber::CameraSubscriber(ImageTransport& image_it, ros::NodeHandle& info_nh,
                                   const std::string& base_topic, uint32_t queue_size,
                                   const Callback& callback, const ros::VoidPtr& tracked_object,
                                   const TransportHints& transport_hints)
  : impl_(new Impl(queue_size))
{
  std::string info_topic = getCameraInfoTopic(base_topic);
  impl_->image_sub_.subscribe(image_it, base_topic, queue_size, transport_hints);
  impl_->info_sub_ .subscribe(info_nh, info_topic, queue_size, transport_hints.getRosHints());
  impl_->sync_.connectInput(impl_->image_sub_, impl_->info_sub_);
  // need for Boost.Bind here is kind of broken
  impl_->sync_.registerCallback(boost::bind(callback, _1, _2));
}

std::string CameraSubscriber::getTopic() const
{
  if (impl_) return impl_->image_sub_.getTopic();
  return std::string();
}

std::string CameraSubscriber::getInfoTopic() const
{
  if (impl_) return impl_->info_sub_.getTopic();
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

#include "image_transport/camera_image_subscriber.h"
#include "image_transport/image_subscriber_filter.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

namespace image_transport {

struct CameraImageSubscriber::Impl
{
  Impl(uint32_t queue_size)
    : sync_(queue_size)
  {}
  
  std::string image_topic_;
  std::string info_topic_;
  ImageSubscriberFilter image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> sync_;
};

CameraImageSubscriber::CameraImageSubscriber()
{
}

CameraImageSubscriber::~CameraImageSubscriber()
{
}

void CameraImageSubscriber::subscribe(ros::NodeHandle& nh, const std::string& image_topic,
                                      uint32_t queue_size, const Callback& callback,
                                      const ros::VoidPtr& tracked_object,
                                      const ros::TransportHints& transport_hints)
{
  std::string info_topic = image_topic.substr(0, image_topic.rfind('/')) + "/camera_info";
  subscribe(nh, image_topic, info_topic, queue_size, callback, tracked_object, transport_hints);
}
  
void CameraImageSubscriber::subscribe(ros::NodeHandle& nh,
                                      const std::string& image_topic, const std::string& info_topic,
                                      uint32_t queue_size, const Callback& callback,
                                      const ros::VoidPtr& tracked_object,
                                      const ros::TransportHints& transport_hints)
{
  shutdown();

  impl_.reset(new Impl(queue_size));
  impl_->image_sub_.subscribe(nh, image_topic, queue_size, transport_hints);
  impl_->info_sub_.subscribe(nh, info_topic, queue_size, transport_hints);
  impl_->sync_.connectInput(impl_->image_sub_, impl_->info_sub_);
  // @todo: need for Boost.Bind here is pretty broken
  impl_->sync_.registerCallback(boost::bind(callback, _1, _2));
}

std::string CameraImageSubscriber::getImageTopic() const
{
  return impl_->image_topic_;
}

std::string CameraImageSubscriber::getInfoTopic() const
{
  return impl_->info_topic_;
}

void CameraImageSubscriber::shutdown()
{
  impl_.reset();
}

} //namespace image_transport

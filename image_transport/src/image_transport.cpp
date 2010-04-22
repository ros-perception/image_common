#include "image_transport/image_transport.h"
#include "image_transport/publisher_plugin.h"
#include "image_transport/subscriber_plugin.h"
#include <pluginlib/class_loader.h>
#include <boost/make_shared.hpp>

namespace image_transport {

struct ImageTransport::Impl
{
  ros::NodeHandle nh_;
  PubLoaderPtr pub_loader_;
  SubLoaderPtr sub_loader_;
  
  Impl(const ros::NodeHandle& nh)
    : nh_(nh),
      pub_loader_( boost::make_shared<PubLoader>("image_transport", "image_transport::PublisherPlugin") ),
      sub_loader_( boost::make_shared<SubLoader>("image_transport", "image_transport::SubscriberPlugin") )
  {
  }
};

ImageTransport::ImageTransport(const ros::NodeHandle& nh)
  : impl_(new Impl(nh))
{
}

ImageTransport::~ImageTransport()
{
}

Publisher ImageTransport::advertise(const std::string& base_topic, uint32_t queue_size, bool latch)
{
  return advertise(base_topic, queue_size, SubscriberStatusCallback(),
                   SubscriberStatusCallback(), ros::VoidPtr(), latch);
}

Publisher ImageTransport::advertise(const std::string& base_topic, uint32_t queue_size,
                                    const SubscriberStatusCallback& connect_cb,
                                    const SubscriberStatusCallback& disconnect_cb,
                                    const ros::VoidPtr& tracked_object, bool latch)
{
  return Publisher(impl_->nh_, base_topic, queue_size, connect_cb, disconnect_cb, tracked_object, latch, impl_->pub_loader_);
}

Subscriber ImageTransport::subscribe(const std::string& base_topic, uint32_t queue_size,
                                     const boost::function<void(const sensor_msgs::ImageConstPtr&)>& callback,
                                     const ros::VoidPtr& tracked_object, const TransportHints& transport_hints)
{
  return Subscriber(impl_->nh_, base_topic, queue_size, callback, tracked_object, transport_hints, impl_->sub_loader_);
}

CameraPublisher ImageTransport::advertiseCamera(const std::string& base_topic, uint32_t queue_size, bool latch)
{
  return advertiseCamera(base_topic, queue_size,
                         SubscriberStatusCallback(), SubscriberStatusCallback(),
                         ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(),
                         ros::VoidPtr(), latch);
}

CameraPublisher ImageTransport::advertiseCamera(const std::string& base_topic, uint32_t queue_size,
                                                const SubscriberStatusCallback& image_connect_cb,
                                                const SubscriberStatusCallback& image_disconnect_cb,
                                                const ros::SubscriberStatusCallback& info_connect_cb,
                                                const ros::SubscriberStatusCallback& info_disconnect_cb,
                                                const ros::VoidPtr& tracked_object, bool latch)
{
  return CameraPublisher(*this, impl_->nh_, base_topic, queue_size, image_connect_cb, image_disconnect_cb,
                         info_connect_cb, info_disconnect_cb, tracked_object, latch);
}

CameraSubscriber ImageTransport::subscribeCamera(const std::string& base_topic, uint32_t queue_size,
                                                 const CameraSubscriber::Callback& callback,
                                                 const ros::VoidPtr& tracked_object,
                                                 const TransportHints& transport_hints)
{
  return CameraSubscriber(*this, impl_->nh_, base_topic, queue_size, callback, tracked_object, transport_hints);
}

} //namespace image_transport

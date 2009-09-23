#include "image_transport/image_transport.h"

namespace image_transport {

ImageTransport::ImageTransport(const ros::NodeHandle& nh)
  : nh_(nh)
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
  return Publisher(nh_, base_topic, queue_size, connect_cb, disconnect_cb, tracked_object, latch);
}

Subscriber ImageTransport::subscribe(const std::string& base_topic, uint32_t queue_size,
                                     const boost::function<void(const sensor_msgs::ImageConstPtr&)>& callback,
                                     const ros::VoidPtr& tracked_object, const TransportHints& transport_hints)
{
  return Subscriber(nh_, base_topic, queue_size, callback, tracked_object, transport_hints);
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
  return CameraPublisher(nh_, base_topic, queue_size, image_connect_cb, image_disconnect_cb,
                         info_connect_cb, info_disconnect_cb, tracked_object, latch);
}

CameraSubscriber ImageTransport::subscribeCamera(const std::string& base_topic, uint32_t queue_size,
                                                 const CameraSubscriber::Callback& callback,
                                                 const ros::VoidPtr& tracked_object,
                                                 const TransportHints& transport_hints)
{
  return CameraSubscriber(nh_, base_topic, queue_size, callback, tracked_object, transport_hints);
}

} //namespace image_transport

#include "image_transport/raw_subscriber.h"

namespace image_transport {

RawSubscriber::RawSubscriber()
{
}
  
RawSubscriber::~RawSubscriber()
{
}

void RawSubscriber::subscribe(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                              const boost::function<void(const sensor_msgs::ImageConstPtr&)>& callback,
                              const ros::VoidPtr& tracked_object,
                              const ros::TransportHints& transport_hints)
{
  sub_ = nh.subscribe<sensor_msgs::Image>(base_topic, queue_size, callback,
                                          tracked_object, transport_hints);
}

std::string RawSubscriber::getTopic() const
{
  return sub_.getTopic();
}

void RawSubscriber::shutdown()
{
  sub_.shutdown();
}

} //namespace image_transport

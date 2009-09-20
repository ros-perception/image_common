#include "image_transport/raw_publisher.h"

namespace image_transport {

RawPublisher::~RawPublisher() {}

std::string RawPublisher::getTransportName() const
{
  return "raw";
}

void RawPublisher::advertise(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                             const ros::SubscriberStatusCallback& connect_cb,
                             const ros::SubscriberStatusCallback& disconnect_cb,
                             const ros::VoidPtr& tracked_object, bool latch)
{
  pub_ = nh.advertise<sensor_msgs::Image>(base_topic, queue_size, connect_cb, disconnect_cb,
                                          tracked_object, latch);
}

uint32_t RawPublisher::getNumSubscribers() const
{
  return pub_.getNumSubscribers();
}

std::string RawPublisher::getTopic() const
{
  return pub_.getTopic();
}

void RawPublisher::publish(const sensor_msgs::Image& message) const
{
  pub_.publish(message);
}

void RawPublisher::shutdown()
{
  pub_.shutdown();
}

} //namespace image_transport

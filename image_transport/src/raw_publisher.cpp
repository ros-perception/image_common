#include "image_transport/raw_publisher.h"

namespace image_transport {

RawPublisher::~RawPublisher() {}

std::string RawPublisher::getTransportType() const
{
  return "raw";
}

std::string RawPublisher::getDefaultTopic(const std::string& base_topic) const
{
  return base_topic;
}

void RawPublisher::advertise(ros::NodeHandle& nh, const std::string& topic,
                                 uint32_t queue_size, bool latch)
{
  pub_ = nh.advertise<sensor_msgs::Image>(topic, queue_size, latch);
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

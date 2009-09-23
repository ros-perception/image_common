#include "image_transport/raw_publisher.h"

namespace image_transport {

RawPublisher::~RawPublisher() {}

std::string RawPublisher::getTransportName() const
{
  return "raw";
}

void RawPublisher::advertiseImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                                 const SubscriberStatusCallback& connect_cb,
                                 const SubscriberStatusCallback& disconnect_cb,
                                 const ros::VoidPtr& tracked_object, bool latch)
{
  pub_ = nh.advertise<sensor_msgs::Image>(base_topic, queue_size, bindCB(connect_cb),
                                          bindCB(disconnect_cb), tracked_object, latch);
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

ros::SubscriberStatusCallback RawPublisher::bindCB(const SubscriberStatusCallback& user_cb)
{
  if (user_cb)
    return boost::bind(&RawPublisher::subscriberCB, this, _1, user_cb);
  else
    return ros::SubscriberStatusCallback();
}

void RawPublisher::subscriberCB(const ros::SingleSubscriberPublisher& ros_pub,
                                const SubscriberStatusCallback& user_cb)
{
  // Bind publish function to ros::SingleSubscriberPublisher::publish
  typedef void (ros::SingleSubscriberPublisher::*PublishMemFn)(const ros::Message&) const;
  PublishMemFn pub_mem_fn = &ros::SingleSubscriberPublisher::publish;
  SingleSubscriberPublisher ssp(ros_pub.getSubscriberCallerID(), getTopic(),
                                boost::bind(&RawPublisher::getNumSubscribers, this),
                                boost::bind(pub_mem_fn, &ros_pub, _1));
  user_cb(ssp);
}

} //namespace image_transport

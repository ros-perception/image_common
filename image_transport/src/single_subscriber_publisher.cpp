#include "image_transport/single_subscriber_publisher.h"
#include "image_transport/publisher.h"

namespace image_transport {

SingleSubscriberPublisher::SingleSubscriberPublisher(const std::string& caller_id, const std::string& topic,
                                                     const GetNumSubscribersFn& num_subscribers_fn,
                                                     const PublishFn& publish_fn)
  : caller_id_(caller_id), topic_(topic),
    num_subscribers_fn_(num_subscribers_fn),
    publish_fn_(publish_fn)
{
}

std::string SingleSubscriberPublisher::getSubscriberName() const
{
  return caller_id_;
}

std::string SingleSubscriberPublisher::getTopic() const
{
  return topic_;
}

uint32_t SingleSubscriberPublisher::getNumSubscribers() const
{
  return num_subscribers_fn_();
}

void SingleSubscriberPublisher::publish(const sensor_msgs::Image& message) const
{
  publish_fn_(message);
}

void SingleSubscriberPublisher::publish(const sensor_msgs::ImageConstPtr& message) const
{
  publish_fn_(*message);
}

} //namespace image_transport

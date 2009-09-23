#ifndef IMAGE_TRANSPORT_SINGLE_SUBSCRIBER_PUBLISHER
#define IMAGE_TRANSPORT_SINGLE_SUBSCRIBER_PUBLISHER

#include <boost/noncopyable.hpp>
#include <boost/function.hpp>
#include <sensor_msgs/Image.h>

namespace image_transport {

/**
 * \brief Allows publication of an image to a single subscriber. Only available inside
 * subscriber connection callbacks.
 */
class SingleSubscriberPublisher : boost::noncopyable
{
public:
  typedef boost::function<uint32_t()> GetNumSubscribersFn;
  typedef boost::function<void(const sensor_msgs::Image&)> PublishFn;
  
  SingleSubscriberPublisher(const std::string& caller_id, const std::string& topic,
                            const GetNumSubscribersFn& num_subscribers_fn,
                            const PublishFn& publish_fn);
  
  std::string getSubscriberCallerID() const;

  std::string getTopic() const;

  uint32_t getNumSubscribers() const;

  void publish(const sensor_msgs::Image& message) const;
  void publish(const sensor_msgs::ImageConstPtr& message) const;

private:
  std::string caller_id_;
  std::string topic_;
  GetNumSubscribersFn num_subscribers_fn_;
  PublishFn publish_fn_;

  friend class Publisher; // to get publish_fn_ directly
};

typedef boost::function<void(const SingleSubscriberPublisher&)> SubscriberStatusCallback;

} //namespace image_transport

#endif

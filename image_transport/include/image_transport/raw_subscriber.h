#include "image_transport/subscriber_plugin.h"

namespace image_transport {

class RawSubscriber : public SubscriberPlugin
{
public:
  RawSubscriber();
  
  virtual ~RawSubscriber();

  virtual void subscribe(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                         const boost::function<void(const sensor_msgs::ImageConstPtr&)>& callback,
                         const ros::VoidPtr& tracked_object,
                         const ros::TransportHints& transport_hints);
  
  virtual std::string getTopic() const;

  virtual void shutdown();

private:
  ros::Subscriber sub_;
};

} //namespace image_transport

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <boost/noncopyable.hpp>

namespace image_transport {

class PublisherPlugin : boost::noncopyable
{
public:
  virtual ~PublisherPlugin() {}

  virtual std::string getTransportType() const = 0;

  virtual std::string getDefaultTopic(const std::string& base_topic) const
  {
    return base_topic + "_" + getTransportType();
  }
  
  // @todo: overloads of advertise
  virtual void advertise(ros::NodeHandle& nh, const std::string& topic,
                         uint32_t queue_size, bool latch) = 0;

  virtual uint32_t getNumSubscribers() const = 0;
  virtual std::string getTopic() const = 0;

  virtual void publish(const sensor_msgs::Image& message) const = 0;
  virtual void publish(const sensor_msgs::ImageConstPtr& message) const
  {
    publish(*message);
  }

  virtual void shutdown() = 0;

  static std::string getLookupName(const std::string& transport_type)
  {
    return transport_type + "_pub";
  }
};

} //namespace image_transport

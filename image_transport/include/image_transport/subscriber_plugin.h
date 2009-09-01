#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <boost/noncopyable.hpp>

namespace image_transport {

class SubscriberPlugin : boost::noncopyable
{
public:
  typedef boost::function<void(const sensor_msgs::ImageConstPtr&)> Callback;
  
  virtual ~SubscriberPlugin() {}

  virtual void subscribe(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                         const boost::function<void(const sensor_msgs::ImageConstPtr&)>& callback,
                         const ros::VoidPtr& tracked_object,
                         const ros::TransportHints& transport_hints) = 0;
  // @todo: other subscribe overloads, which default to calling above version
  
  virtual std::string getTopic() const = 0;

  virtual void shutdown() = 0;

  static std::string getLookupName(const std::string& transport_type)
  {
    return transport_type + "_sub";
  }
};

} //namespace image_transport

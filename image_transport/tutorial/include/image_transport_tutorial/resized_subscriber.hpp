#include <image_transport/simple_subscriber_plugin.hpp>
#include <image_transport_tutorial/ResizedImage.h>

class ResizedSubscriber : public image_transport::SimpleSubscriberPlugin<image_transport_tutorial::ResizedImage>
{
public:
  virtual ~ResizedSubscriber() {}

  virtual std::string getTransportName() const
  {
    return "resized";
  }

  void subscribeImpl(
    rclcpp::Node * node,
    const std::string & base_topic,
    const Callback & callback,
    rmw_qos_profile_t custom_qos,
    rclcpp::SubscriptionOptions options) override
  {
    this->subscribeImplWithOptions(node, base_topic, callback, custom_qos, options);
  }
protected:
  virtual void internalCallback(const typename image_transport_tutorial::ResizedImage::ConstPtr& message,
                                const Callback& user_cb);
};

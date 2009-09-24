#ifndef IMAGE_TRANSPORT_SIMPLE_SUBSCRIBER_PLUGIN_H
#define IMAGE_TRANSPORT_SIMPLE_SUBSCRIBER_PLUGIN_H

#include "image_transport/subscriber_plugin.h"
#include <boost/scoped_ptr.hpp>

namespace image_transport {

template <class M>
class SimpleSubscriberPlugin : public SubscriberPlugin
{
public:
  virtual ~SimpleSubscriberPlugin() {}

  virtual std::string getTopic() const
  {
    return simple_impl_->sub_.getTopic();
  }

  virtual void shutdown()
  {
    simple_impl_->sub_.shutdown();
  }

protected:
  virtual void internalCallback(const typename M::ConstPtr& message, const Callback& user_cb) = 0;

  virtual std::string getTopicToSubscribe(const std::string& base_topic) const
  {
    return base_topic + "/" + getTransportName();
  }
  
  virtual void subscribeImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                             const Callback& callback, const ros::VoidPtr& tracked_object,
                             const ros::TransportHints& transport_hints)
  {
    simple_impl_.reset(new SimpleSubscriberPluginImpl(nh));

    simple_impl_->sub_ = nh.subscribe<M>(getTopicToSubscribe(base_topic), queue_size,
                                         boost::bind(&SimpleSubscriberPlugin::internalCallback, this, _1, callback),
                                         tracked_object, transport_hints);
  }

private:
  struct SimpleSubscriberPluginImpl
  {
    SimpleSubscriberPluginImpl(ros::NodeHandle& nh)
      : nh_(nh)
    {
    }
    
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    //boost::function<void(const sensor_msgs::ImageConstPtr&)> user_cb_;
  };
  
  boost::scoped_ptr<SimpleSubscriberPluginImpl> simple_impl_;
};

} //namespace image_transport

#endif

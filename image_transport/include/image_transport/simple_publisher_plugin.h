#ifndef IMAGE_TRANSPORT_SIMPLE_PUBLISHER_PLUGIN_H
#define IMAGE_TRANSPORT_SIMPLE_PUBLISHER_PLUGIN_H

#include "image_transport/publisher_plugin.h"
#include <boost/scoped_ptr.hpp>

namespace image_transport {

template <class M>
class SimplePublisherPlugin : public PublisherPlugin
{
public:
  virtual ~SimplePublisherPlugin() {}

  virtual uint32_t getNumSubscribers() const
  {
    return simple_impl_->pub_.getNumSubscribers();
  }

  virtual std::string getTopic() const
  {
    return simple_impl_->pub_.getTopic();
  }

  virtual void publish(const sensor_msgs::Image& message) const
  {
    publish(message, bindInternalPublisher(simple_impl_->pub_));
  }

  virtual void shutdown()
  {
    simple_impl_->pub_.shutdown();
  }

protected:
  virtual void advertiseImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                             const SubscriberStatusCallback& connect_cb,
                             const SubscriberStatusCallback& disconnect_cb,
                             const ros::VoidPtr& tracked_object, bool latch)
  {
    simple_impl_.reset(new SimplePublisherPluginImpl(nh));
    simple_impl_->pub_ = nh.advertise<M>(getTopicToAdvertise(base_topic), queue_size,
                                         bindCB(connect_cb), bindCB(disconnect_cb),
                                         tracked_object, latch);
  }

  //! Generic function for publishing the internal message type.
  typedef boost::function<void(const M&)> PublishFn;

  /**
   * \brief Publish an image using the specified publish function. Must be implemented by
   * the subclass.
   */
  virtual void publish(const sensor_msgs::Image& message, const PublishFn& publish_fn) const = 0;

  virtual std::string getTopicToAdvertise(const std::string& base_topic) const
  {
    return base_topic + "/" + getTransportName();
  }

  const ros::NodeHandle& nh() const
  {
    return simple_impl_->nh_;
  }

private:
  struct SimplePublisherPluginImpl
  {
    SimplePublisherPluginImpl(ros::NodeHandle& nh)
      : nh_(nh)
    {
    }
    
    ros::NodeHandle nh_;
    ros::Publisher pub_;
  };
  
  boost::scoped_ptr<SimplePublisherPluginImpl> simple_impl_;

  ros::SubscriberStatusCallback bindCB(const SubscriberStatusCallback& user_cb)
  {
    if (user_cb)
      return boost::bind(&SimplePublisherPlugin::subscriberCB, this, _1, user_cb);
    else
      return ros::SubscriberStatusCallback();
  }

  void subscriberCB(const ros::SingleSubscriberPublisher& ros_ssp,
                    const SubscriberStatusCallback& user_cb)
  {
    // Invoke user callback with the single subscriber publisher
    SingleSubscriberPublisher ssp(ros_ssp.getSubscriberCallerID(), getTopic(),
                                  boost::bind(&SimplePublisherPlugin::getNumSubscribers, this),
                                  bindInternalPublisher(ros_ssp));
    user_cb(ssp);
  }

  typedef boost::function<void(const sensor_msgs::Image&)> ImagePublishFn;
  template <class PubT>
  ImagePublishFn bindInternalPublisher(const PubT& pub) const
  {
    // Bind PubT::publish(const Message&) as PublishFn
    typedef void (PubT::*InternalPublishMemFn)(const ros::Message&) const;
    InternalPublishMemFn internal_pub_mem_fn = &PubT::publish;
    PublishFn publish_fn = boost::bind(internal_pub_mem_fn, &pub, _1);
    
    // Bind publish_fn to the subclass-implemented this->publish(Image, PublishFn)
    typedef void (SimplePublisherPlugin::*PublishMemFn)(const sensor_msgs::Image&, const PublishFn&) const;
    PublishMemFn pub_mem_fn = &SimplePublisherPlugin::publish;
    return boost::bind(pub_mem_fn, this, _1, publish_fn);
  }
};

} //namespace image_transport

#endif

#ifndef IMAGE_TRANSPORT_IMAGE_TRANSPORT_H
#define IMAGE_TRANSPORT_IMAGE_TRANSPORT_H

#include "image_transport/publisher.h"
#include "image_transport/subscriber.h"
#include "image_transport/transport_hints.h"

namespace image_transport {

class ImageTransport
{
public:
  ImageTransport(const ros::NodeHandle& nh);

  /*!
   * \brief Advertise a topic, simple version.
   */
  Publisher advertise(const std::string& base_topic, uint32_t queue_size, bool latch = false);

  /*!
   * \brief Advertise a topic with subcriber status callbacks.
   */
  Publisher advertise(const std::string& base_topic, uint32_t queue_size,
                      const ros::SubscriberStatusCallback& connect_cb,
                      const ros::SubscriberStatusCallback& disconnect_cb = ros::SubscriberStatusCallback(),
                      const ros::VoidPtr& tracked_object = ros::VoidPtr(), bool latch = false);

  /**
   * \brief Subscribe to a topic, version for arbitrary boost::function object.
   */
  Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
                       const boost::function<void(const sensor_msgs::ImageConstPtr&)>& callback,
                       const ros::VoidPtr& tracked_object = ros::VoidPtr(),
                       const TransportHints& transport_hints = TransportHints());

  /**
   * \brief Subscribe to a topic, version for bare function.
   */
  Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
                       void(*fp)(const sensor_msgs::ImageConstPtr&),
                       const TransportHints& transport_hints = TransportHints())
  {
    return subscribe(base_topic, queue_size,
                     boost::function<void(const sensor_msgs::ImageConstPtr&)>(fp),
                     ros::VoidPtr(), transport_hints);
  }

  /**
   * \brief Subscribe to a topic, version for class member function with bare pointer.
   */
  template<class T>
  Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
                       void(T::*fp)(const sensor_msgs::ImageConstPtr&), T* obj,
                       const TransportHints& transport_hints = TransportHints())
  {
    return subscribe(base_topic, queue_size, boost::bind(fp, obj, _1), ros::VoidPtr(), transport_hints);
  }

  /**
   * \brief Subscribe to a topic, version for class member function with shared_ptr.
   */
  template<class T>
  Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
                       void(T::*fp)(const sensor_msgs::ImageConstPtr&),
                       const boost::shared_ptr<T>& obj,
                       const TransportHints& transport_hints = TransportHints())
  {
    return subscribe(base_topic, queue_size, boost::bind(fp, obj.get(), _1), obj, transport_hints);
  }

private:
  ros::NodeHandle nh_;
};

} //namespace image_transport

#endif

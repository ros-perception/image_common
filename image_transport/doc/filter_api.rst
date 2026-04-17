Message Filter Integration
==========================

``image_transport`` provides :cpp:class:`image_transport::SubscriberFilter`, a
drop-in replacement for ``message_filters::Subscriber`` that works with the
full suite of ``message_filters`` synchronisation policies (e.g.
``TimeSynchronizer``, ``ApproximateTime``).

SubscriberFilter
----------------

:cpp:class:`image_transport::SubscriberFilter` is a
``message_filters``-compatible subscriber that decodes compressed image streams
before delivering them to the filter chain.

See the full :ref:`exhale_class_classimage__transport_1_1SubscriberFilter` API reference.

Usage Example
-------------

The following example synchronises an image with a ``PointCloud2`` message
using ``message_filters::TimeSynchronizer``.

.. code-block:: cpp

   #include <rclcpp/rclcpp.hpp>
   #include <image_transport/subscriber_filter.hpp>
   #include <message_filters/subscriber.h>
   #include <message_filters/time_synchronizer.h>
   #include <sensor_msgs/msg/point_cloud2.hpp>

   class SyncNode : public rclcpp::Node
   {
   public:
     SyncNode() : Node("sync_node")
     {
       image_sub_.subscribe(shared_from_this(), "camera/image_raw");
       cloud_sub_.subscribe(shared_from_this(), "points");

       sync_ = std::make_shared<
         message_filters::TimeSynchronizer<
           sensor_msgs::msg::Image,
           sensor_msgs::msg::PointCloud2>>(image_sub_, cloud_sub_, 10);
       sync_->registerCallback(
         std::bind(&SyncNode::callback, this,
           std::placeholders::_1, std::placeholders::_2));
     }

   private:
     void callback(
       const sensor_msgs::msg::Image::ConstSharedPtr & img,
       const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
     {
       RCLCPP_INFO(get_logger(), "Synced frame at %u ns",
         img->header.stamp.nanosec);
     }

     image_transport::SubscriberFilter image_sub_;
     message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
     std::shared_ptr<message_filters::TimeSynchronizer<
       sensor_msgs::msg::Image,
       sensor_msgs::msg::PointCloud2>> sync_;
   };

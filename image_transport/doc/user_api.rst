User API
========

These are the classes and free functions intended for everyday use in
camera drivers and image-processing nodes.

ImageTransport
--------------

:cpp:class:`image_transport::ImageTransport` is the entry point.  Bind it to
a node and use it to create publishers and subscribers.

See the full :ref:`exhale_class_classimage__transport_1_1ImageTransport` API reference.

Publisher
---------

:cpp:class:`image_transport::Publisher` is the handle returned by
``ImageTransport::advertise()``.

See the full :ref:`exhale_class_classimage__transport_1_1Publisher` API reference.

Subscriber
----------

:cpp:class:`image_transport::Subscriber` is the handle returned by
``ImageTransport::subscribe()``.

See the full :ref:`exhale_class_classimage__transport_1_1Subscriber` API reference.

TransportHints
--------------

:cpp:class:`image_transport::TransportHints` controls which transport plugin
is selected for a subscriber.

See the full :ref:`exhale_class_classimage__transport_1_1TransportHints` API reference.

SingleSubscriberPublisher
-------------------------

:cpp:class:`image_transport::SingleSubscriberPublisher` is passed to the
``connect_cb`` of a ``Publisher`` when a new subscriber arrives.

See the full :ref:`exhale_class_classimage__transport_1_1SingleSubscriberPublisher` API reference.

Free Functions
--------------

The free functions ``create_publisher``, ``create_subscription``,
``create_camera_publisher``, and ``create_camera_subscription`` are available
in the ``image_transport`` namespace. See the C++ API reference for the full
overload set.


Exceptions
----------

:cpp:class:`image_transport::Exception` is the base exception class.

:cpp:class:`image_transport::TransportLoadException` is thrown when a
transport plugin cannot be loaded.

See the full API reference:
:ref:`exhale_class_classimage__transport_1_1Exception`,
:ref:`exhale_class_classimage__transport_1_1TransportLoadException`.

Usage Example
-------------

.. code-block:: cpp

   #include <rclcpp/rclcpp.hpp>
   #include <image_transport/image_transport.hpp>
   #include <sensor_msgs/msg/image.hpp>

   class MyNode : public rclcpp::Node
   {
   public:
     MyNode() : Node("my_node")
     {
       // Create ImageTransport bound to this node
       it_ = std::make_shared<image_transport::ImageTransport>(
         shared_from_this());

       // Publish on base topic — all transports are advertised automatically
       pub_ = it_->advertise("camera/image", 1);

       // Subscribe — honours the 'image_transport' parameter
       sub_ = it_->subscribe(
         "camera/image", 1,
         std::bind(&MyNode::imageCb, this, std::placeholders::_1));
     }

   private:
     void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
     {
       RCLCPP_INFO(get_logger(), "Received image %ux%u", msg->width, msg->height);
       pub_.publish(msg);
     }

     std::shared_ptr<image_transport::ImageTransport> it_;
     image_transport::Publisher pub_;
     image_transport::Subscriber sub_;
   };

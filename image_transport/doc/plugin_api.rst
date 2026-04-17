Writing a Transport Plugin
==========================

A transport plugin is a pair of ``pluginlib`` plugins — one publisher and one
subscriber — that negotiate how image data is serialised on the wire. The
built-in ``raw`` transport (see ``RawPublisher`` / ``RawSubscriber``) is the
simplest possible example.

Plugin Base Classes
-------------------

PublisherPlugin
^^^^^^^^^^^^^^^

:cpp:class:`image_transport::PublisherPlugin` is the abstract base class for
all publisher-side transport plugins.

See the full :ref:`exhale_class_classimage__transport_1_1PublisherPlugin` API reference.

SubscriberPlugin
^^^^^^^^^^^^^^^^

:cpp:class:`image_transport::SubscriberPlugin` is the abstract base class for
all subscriber-side transport plugins.

See the full :ref:`exhale_class_classimage__transport_1_1SubscriberPlugin` API reference.

Convenience Base Classes
------------------------

Most transports communicate over a single ROS 2 topic using a
transport-specific message type. In that common case, derive from the
``Simple`` variants to avoid boilerplate.

SimplePublisherPlugin
^^^^^^^^^^^^^^^^^^^^^

:cpp:class:`image_transport::SimplePublisherPlugin` provides a ready-made
publisher plugin that manages a single typed topic subscription internally.

See the full :ref:`exhale_class_classimage__transport_1_1SimplePublisherPlugin` API reference.

SimpleSubscriberPlugin
^^^^^^^^^^^^^^^^^^^^^^

:cpp:class:`image_transport::SimpleSubscriberPlugin` provides a ready-made
subscriber plugin that manages a single typed topic subscription internally.

See the full :ref:`exhale_class_classimage__transport_1_1SimpleSubscriberPlugin` API reference.

Built-in Raw Transport
----------------------

RawPublisher
^^^^^^^^^^^^

:cpp:class:`image_transport::RawPublisher` is the built-in raw transport
publisher, which publishes ``sensor_msgs/msg/Image`` directly without encoding.

See the full :ref:`exhale_class_classimage__transport_1_1RawPublisher` API reference.

RawSubscriber
^^^^^^^^^^^^^

:cpp:class:`image_transport::RawSubscriber` is the built-in raw transport
subscriber.

See the full :ref:`exhale_class_classimage__transport_1_1RawSubscriber` API reference.

Step-by-Step Guide
------------------

1. **Define the transport message** (optional). If the compressed format needs
   its own message type, add it to a separate ``_msgs`` package. Otherwise
   reuse an existing type.

2. **Implement the publisher plugin.** Derive from
   :cpp:class:`image_transport::SimplePublisherPlugin` and override
   ``publish()`` to encode the raw ``sensor_msgs::msg::Image`` into your
   transport-specific message.

   .. code-block:: cpp

      class MyPublisher
        : public image_transport::SimplePublisherPlugin<my_msgs::msg::MyImage>
      {
      public:
        std::string getTransportName() const override { return "my_transport"; }

      protected:
        void publish(
          const sensor_msgs::msg::Image & message,
          const PublishFn & publish_fn) const override
        {
          my_msgs::msg::MyImage encoded;
          // ... encode message into encoded ...
          publish_fn(encoded);
        }
      };

3. **Implement the subscriber plugin.** Derive from
   :cpp:class:`image_transport::SimpleSubscriberPlugin` and override
   ``internalCallback()`` to decode the transport-specific message back into
   a ``sensor_msgs::msg::Image``.

   .. code-block:: cpp

      class MySubscriber
        : public image_transport::SimpleSubscriberPlugin<my_msgs::msg::MyImage>
      {
      public:
        std::string getTransportName() const override { return "my_transport"; }

      protected:
        void internalCallback(
          const my_msgs::msg::MyImage::ConstSharedPtr & msg,
          const Callback & user_cb) override
        {
          auto image = std::make_shared<sensor_msgs::msg::Image>();
          // ... decode msg into image ...
          user_cb(image);
        }
      };

4. **Register the plugins** in your ``CMakeLists.txt`` and export a
   ``pluginlib`` XML manifest so that ``image_transport`` can discover them
   at runtime.

   .. code-block:: xml

      <library path="my_transport_plugins">
        <class name="my_transport/MyPublisher"
               type="my_transport::MyPublisher"
               base_class_type="image_transport::PublisherPlugin">
          <description>My custom image transport publisher.</description>
        </class>
        <class name="my_transport/MySubscriber"
               type="my_transport::MySubscriber"
               base_class_type="image_transport::SubscriberPlugin">
          <description>My custom image transport subscriber.</description>
        </class>
      </library>

5. **Export the XML manifest** from ``package.xml``:

   .. code-block:: xml

      <export>
        <image_transport plugin="${prefix}/my_transport_plugins.xml"/>
      </export>

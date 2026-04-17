Overview
========

``image_transport`` is a ROS 2 library that provides a plugin-based framework
for publishing and subscribing to images. It acts as a transparent layer on
top of standard ROS 2 publishers and subscribers, allowing image data to be
transported in raw or compressed form without any changes to the application
code.

Motivation
----------

Raw images are large. A single 1080p colour frame is roughly 6 MB. Sending
these frames over a network at 30 Hz consumes nearly 1.5 Gbit/s of bandwidth.
``image_transport`` solves this by:

* Advertising a *family* of topics for every base image topic (e.g.
  ``/camera/image_raw``, ``/camera/image_raw/compressed``,
  ``/camera/image_raw/theora``, …).
* Letting subscribers pick the transport that best suits their network
  conditions by setting the ``image_transport`` ROS parameter.
* Keeping the API identical to standard ``rclcpp`` publishers and subscribers
  so that existing code requires minimal changes.

Architecture
------------

.. code-block:: text

   Camera driver
       │
       ▼
   image_transport::Publisher
       ├─ raw topic          ──► raw subscriber
       ├─ compressed topic   ──► compressed subscriber
       └─ theora topic       ──► theora subscriber

The publisher side creates one sub-publisher per installed transport plugin.
The subscriber side picks exactly one transport based on the
``image_transport`` parameter (default: ``raw``).

Plugin system
^^^^^^^^^^^^^

Transport plugins are discovered at runtime via ``pluginlib``. Each plugin
implements either :cpp:class:`image_transport::PublisherPlugin` or
:cpp:class:`image_transport::SubscriberPlugin`. The built-in ``raw`` transport
is provided by :cpp:class:`image_transport::RawPublisher` and
:cpp:class:`image_transport::RawSubscriber`.

A list of all currently loaded transport plugins can be obtained with:

.. code-block:: bash

   ros2 run image_transport list_transports

Dependencies
------------

* ``rclcpp`` — ROS 2 C++ client library
* ``sensor_msgs`` — ``Image`` and ``CameraInfo`` message types
* ``message_filters`` — for :cpp:class:`image_transport::SubscriberFilter`
* ``pluginlib`` — runtime plugin loading
* ``rclcpp_components`` — composable node support (``republish``)

ROS Parameters
--------------

.. list-table::
   :header-rows: 1

   * - Parameter
     - Type
     - Default
     - Description
   * - ``image_transport``
     - string
     - ``"raw"``
     - Transport to use when subscribing. Set per-subscriber via
       :cpp:class:`image_transport::TransportHints`.

Nodes
-----

republish
^^^^^^^^^

Converts an image stream from one transport to another.

.. code-block:: bash

   ros2 run image_transport republish \
     --ros-args \
     -p in_transport:=raw \
     -p out_transport:=compressed \
     --remap in:=/camera/image_raw \
     --remap out:=/camera/image_raw

list_transports
^^^^^^^^^^^^^^^

Prints all transport plugins available in the current environment.

.. code-block:: bash

   ros2 run image_transport list_transports

Overview
========

``camera_info_manager`` provides a C++ class that manages
``sensor_msgs/msg/CameraInfo`` for camera drivers. It handles:

- **Loading** calibration data from a ``file://`` or ``package://`` URL on
  first access (lazy loading).
- **Saving** calibration data to a file in response to a
  ``sensor_msgs/srv/SetCameraInfo`` service call.
- **Advertising** the ``set_camera_info`` service so calibration tools can
  push new calibration data to a running camera driver.

Architecture
------------

A camera driver creates a ``CameraInfoManager`` at startup, passing node
interfaces, a camera name and a calibration URL. The manager advertises the
``set_camera_info`` service and lazy-loads calibration data the first time
``getCameraInfo()`` or ``isCalibrated()`` is called.

.. code-block:: none

   Camera Driver Node
   ├── CameraInfoManager
   │   ├── Reads calibration URL → loads CameraInfo from file/package
   │   ├── Serves  set_camera_info  → saves new calibration to file
   │   └── Returns CameraInfo via getCameraInfo()
   └── Publishes Image + CameraInfo on every frame

``getCameraInfo()`` returns a copy of the calibration data, and it does not
fill in the header of the message. The driver must set the time stamp and the
frame ID, normally to the same values as the header of the corresponding image.

If no calibration is available, all the matrices of the returned message are
zero, which the image pipeline handles as uncalibrated data.
``isCalibrated()`` returns ``true`` when the first element of the camera
matrix ``K`` is not zero.

Calibration URL Schemes
-----------------------

``camera_info_manager`` supports two URL schemes for locating calibration
files. The scheme is not case sensitive.

``file://``
    Absolute path on the local filesystem.

    .. code-block:: none

       file:///home/user/calibrations/camera.yaml

``package://``
    Path relative to the share directory of a ROS 2 package.

    .. code-block:: none

       package://my_camera_driver/config/calibration.yaml

    The file therefore has to be installed by the package, for example with
    ``install(DIRECTORY config DESTINATION share/${PROJECT_NAME})``.

Use ``validateURL()`` to check whether the syntax of a URL is supported. It
does not check that the file or the package exist.

The calibration is read and written with ``camera_calibration_parsers``, which
selects the file format from the extension of the file: ``.yaml`` or ``.yml``
for YAML (recommended), or ``.ini`` for the Videre INI format.

URL Variable Substitution
~~~~~~~~~~~~~~~~~~~~~~~~~

URLs may contain the following variables (resolved in a single pass):

- ``${NAME}`` — replaced with the camera name set at construction time or
  via ``setCameraName()``.
- ``${ROS_HOME}`` — replaced with the value of the ``$ROS_HOME`` environment
  variable, or ``~/.ros`` if unset (``%USERPROFILE%/.ros`` on Windows).

Any other ``${...}`` variable is not resolved, and an error is logged.

Default URL (when none is provided):

.. code-block:: none

   file://${ROS_HOME}/camera_info/${NAME}.yaml

Camera Name
-----------

The camera name is used for two purposes:

1. Substituted for ``${NAME}`` in the calibration URL.
2. Compared against the camera name stored inside the calibration file — a
   warning is logged if they differ. The calibration is used anyway.

A valid camera name consists of alphanumeric characters and underscores.
For uniqueness, consider encoding a GUID or make/model/serial combination.

The name given to the constructor is not validated. ``setCameraName()``
rejects invalid names, and when it accepts the new name the calibration is
loaded again the next time it is needed, because the URL may now resolve to a
different file.

Lazy Loading
------------

No I/O is performed in the constructor.  Calibration data are read on the
first call to ``getCameraInfo()``, ``isCalibrated()``, or an explicit
``loadCameraInfo()``.  Call ``loadCameraInfo()`` early, for example in the
constructor of the driver or in the ``on_configure()`` callback of a lifecycle
node, to surface load errors before streaming begins.

Loading can fail in two different ways:

- If the file does not exist or it cannot be parsed, or if the URL is not
  valid, a message is logged and ``loadCameraInfo()`` returns ``false``. The
  previous calibration, if any, is kept.
- If a ``package://`` URL refers to a package that is not installed, a
  ``std::logic_error`` is thrown. This also applies to the lazy load done by
  ``getCameraInfo()`` and ``isCalibrated()``.

ROS 2 Service
-------------

The manager advertises a ``set_camera_info`` service
(``sensor_msgs/srv/SetCameraInfo``).  Calibration tools such as
``camera_calibration`` send new calibration data via this service; the manager
updates the in-memory ``CameraInfo`` and saves the data to the configured URL,
creating the directories of the path if needed.

The in-memory ``CameraInfo`` is always updated, even if the calibration cannot
be saved. In that case the response has ``success`` set to ``false`` and a
``status_message``.  If the configured URL is not valid, the calibration is
saved to the default URL instead.

The name of the service is ``<ns>/set_camera_info``, where ``ns`` is the last
argument of the constructor:

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - ``ns``
     - Service name, for a node named ``driver`` in the ``/robot`` namespace
   * - ``""`` (default)
     - ``/set_camera_info``
   * - ``"~"``
     - ``/robot/driver/set_camera_info``
   * - ``"left"``
     - ``/robot/left/set_camera_info``
   * - ``"~/left"``
     - ``/robot/driver/left/set_camera_info``

.. warning::

   With the default value the name of the service is absolute: it does not
   depend on the namespace of the node.  Pass a namespace when the node is in
   a namespace, when several drivers run at the same time, or when a driver
   manages several cameras (for example a stereo camera), so that every
   ``CameraInfoManager`` advertises a different service.

``setCameraInfo()`` sets the in-memory ``CameraInfo`` from the driver itself.
Unlike the service, it does not save the calibration.

Lifecycle Node Support
----------------------

The constructor takes node interfaces instead of a node, so it works in the
same way with a ``rclcpp::Node`` and with a
``rclcpp_lifecycle::LifecycleNode``:

.. code-block:: cpp

   auto cim = std::make_shared<camera_info_manager::CameraInfoManager>(
     node->get_node_base_interface(),
     node->get_node_services_interface(),
     node->get_node_logging_interface(),
     "my_camera",
     "package://my_driver/config/camera.yaml");

Thread Safety
-------------

The public methods can be called from several threads. The internal state is
protected by a mutex, which is never held during I/O.

Dependencies
------------

- ``rclcpp`` — node interfaces and service server.
- ``sensor_msgs`` — ``CameraInfo`` and ``SetCameraInfo`` types.
- ``camera_calibration_parsers`` — YAML / INI calibration file I/O.
- ``ament_index_cpp`` — resolves ``package://`` URLs to filesystem paths.
- ``rcpputils`` — utility helpers.

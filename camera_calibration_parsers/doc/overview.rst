Overview
========

``camera_calibration_parsers`` provides C++ (and Python) functions for reading
and writing camera calibration data in two human-readable file formats:

- **YAML** — the format produced by the ``camera_calibration`` tool and the
  default format for ``camera_info_manager``.
- **Videre INI** — a legacy format used by older Videre stereo cameras and
  still encountered in the field.

The high-level API in ``parse.hpp`` detects the format automatically from
the file extension, so most callers never need the format-specific headers.

API Layers
----------

.. code-block:: none

   parse.hpp          ← recommended: auto-detects format from file extension
   ├── parse_yml.hpp  ← YAML-specific stream and file overloads
   └── parse_ini.hpp  ← Videre INI-specific stream and file overloads

Each layer exposes three operation types:

- **write** — serialise a ``sensor_msgs/msg/CameraInfo`` to a stream or file.
- **read** — deserialise from a stream or file into a ``CameraInfo``.
- **parse** — deserialise from an in-memory string buffer.

File Formats
------------

YAML
~~~~

Produced and consumed by the ROS 2 ``camera_calibration`` package.  Files
typically carry a ``.yaml`` or ``.yml`` extension and look like:

.. code-block:: yaml

   image_width: 640
   image_height: 480
   camera_name: my_camera
   camera_matrix:
     rows: 3
     cols: 3
     data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
   distortion_model: plumb_bob
   distortion_coefficients:
     rows: 1
     cols: 5
     data: [k1, k2, p1, p2, k3]
   rectification_matrix:
     rows: 3
     cols: 3
     data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
   projection_matrix:
     rows: 3
     cols: 4
     data: [fx', 0, cx', Tx, 0, fy', cy', Ty, 0, 0, 1, 0]

Videre INI
~~~~~~~~~~

A legacy format with a ``.ini`` extension, originally used by Videre Design
stereo cameras.  The parser handles the subset commonly found in open-source
camera drivers.

Python Bindings
---------------

A thin Python wrapper is available via the ``camera_calibration_parsers``
Python module:

.. code-block:: python

   from camera_calibration_parsers import readCalibration

   result = readCalibration('/path/to/camera.yaml')
   if result is not None:
       camera_name, camera_info = result

Dependencies
------------

- ``sensor_msgs`` — ``CameraInfo`` message type.
- ``rclcpp`` — logging.
- ``yaml_cpp_vendor`` — YAML parsing and serialisation.

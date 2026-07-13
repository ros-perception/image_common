API Reference
=============

High-Level API (``parse.hpp``)
-------------------------------

The recommended entry point for most callers.  The format is inferred from
the file extension (``.yaml`` / ``.yml`` → YAML, ``.ini`` → Videre INI).

- :cpp:func:`~camera_calibration_parsers::writeCalibration` — write to a file (format inferred from extension)
- :cpp:func:`~camera_calibration_parsers::readCalibration` — read from a file (format auto-detected)
- :cpp:func:`~camera_calibration_parsers::parseCalibration` — parse from an in-memory string

Browse the full namespace:
:ref:`namespace_camera_calibration_parsers`.

YAML API (``parse_yml.hpp``)
-----------------------------

Format-specific overloads for YAML.  Use these when reading from or writing
to an arbitrary stream rather than a named file.

- :cpp:func:`~camera_calibration_parsers::writeCalibrationYml` — write to an output stream or a named file (overloaded)
- :cpp:func:`~camera_calibration_parsers::readCalibrationYml` — read from an input stream or a named file (overloaded)
- :cpp:func:`~camera_calibration_parsers::parseCalibrationYml` — parse from an in-memory string

Videre INI API (``parse_ini.hpp``)
-----------------------------------

Format-specific overloads for the Videre INI format.

- :cpp:func:`~camera_calibration_parsers::writeCalibrationIni` — write to an output stream or a named file (overloaded)
- :cpp:func:`~camera_calibration_parsers::readCalibrationIni` — read from an input stream or a named file (overloaded)
- :cpp:func:`~camera_calibration_parsers::parseCalibrationIni` — parse from an in-memory string

Type Alias
----------

All three headers define:

.. code-block:: cpp

   using CameraInfo = sensor_msgs::msg::CameraInfo;

Usage Examples
--------------

**Read calibration from a file (auto-detect format)**

.. code-block:: cpp

   #include <camera_calibration_parsers/parse.hpp>

   std::string camera_name;
   sensor_msgs::msg::CameraInfo camera_info;

   if (!camera_calibration_parsers::readCalibration(
         "/path/to/camera.yaml", camera_name, camera_info))
   {
     RCLCPP_ERROR(logger, "Failed to load calibration");
   }

**Write calibration to a file**

.. code-block:: cpp

   #include <camera_calibration_parsers/parse.hpp>

   if (!camera_calibration_parsers::writeCalibration(
         "/tmp/calibration.yaml", "my_camera", camera_info))
   {
     RCLCPP_ERROR(logger, "Failed to save calibration");
   }

**Parse calibration from an in-memory string**

.. code-block:: cpp

   #include <camera_calibration_parsers/parse.hpp>

   std::string yaml_data = get_calibration_from_parameter_server();
   std::string camera_name;
   sensor_msgs::msg::CameraInfo camera_info;

   if (!camera_calibration_parsers::parseCalibration(
         yaml_data, "yml", camera_name, camera_info))
   {
     RCLCPP_ERROR(logger, "Failed to parse calibration string");
   }

**Write to a stream (YAML)**

.. code-block:: cpp

   #include <camera_calibration_parsers/parse_yml.hpp>
   #include <sstream>

   std::ostringstream oss;
   camera_calibration_parsers::writeCalibrationYml(oss, "my_camera", camera_info);
   std::string yaml_string = oss.str();

**Read from a stream (INI)**

.. code-block:: cpp

   #include <camera_calibration_parsers/parse_ini.hpp>
   #include <fstream>

   std::ifstream ifs("/path/to/camera.ini");
   std::string camera_name;
   sensor_msgs::msg::CameraInfo camera_info;
   camera_calibration_parsers::readCalibrationIni(ifs, camera_name, camera_info);

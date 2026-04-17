API Reference
=============

High-Level API (``parse.hpp``)
-------------------------------

The recommended entry point for most callers.  The format is inferred from
the file extension (``.yaml`` / ``.yml`` → YAML, ``.ini`` → Videre INI).

- :ref:`exhale_function_namespacecamera__calibration__parsers_1ab1426265c50ad5f1d2812ab900e201ec` — ``writeCalibration``
- :ref:`exhale_function_namespacecamera__calibration__parsers_1a4242ca29511127c36ea71303b13c70cb` — ``readCalibration``
- :ref:`exhale_function_namespacecamera__calibration__parsers_1a75f5ad9712a3de93e8523ac0c81a43b7` — ``parseCalibration``

Browse the full namespace:
:ref:`namespace_camera_calibration_parsers`.

YAML API (``parse_yml.hpp``)
-----------------------------

Format-specific overloads for YAML.  Use these when reading from or writing
to an arbitrary stream rather than a named file.

- :ref:`exhale_function_namespacecamera__calibration__parsers_1afa6d0c688357d7c01d474d01377d532d` — ``writeCalibrationYml(ostream)``
- :ref:`exhale_function_namespacecamera__calibration__parsers_1a3d98138bf170b225af61515296315ef5` — ``readCalibrationYml(istream)``
- :ref:`exhale_function_namespacecamera__calibration__parsers_1ad818651db64b842660e74e72bfc9c714` — ``writeCalibrationYml(filename)``
- :ref:`exhale_function_namespacecamera__calibration__parsers_1aceea3789c0817cf94a2a24a04122f320` — ``readCalibrationYml(filename)``
- :ref:`exhale_function_namespacecamera__calibration__parsers_1ab8b17fb67c52ebe800ff2021f6e6ae95` — ``parseCalibrationYml``

Videre INI API (``parse_ini.hpp``)
-----------------------------------

Format-specific overloads for the Videre INI format.

- :ref:`exhale_function_namespacecamera__calibration__parsers_1a4c25d94528aa8e4e551e2c6e89a53520` — ``writeCalibrationIni(ostream)``
- :ref:`exhale_function_namespacecamera__calibration__parsers_1a3e89f85a374d92d7976087ab2a3f0f68` — ``readCalibrationIni(istream)``
- :ref:`exhale_function_namespacecamera__calibration__parsers_1a0334c84f46853d8c472132166a22e897` — ``writeCalibrationIni(filename)``
- :ref:`exhale_function_namespacecamera__calibration__parsers_1afaae91efdebd11870baf40459c46e141` — ``readCalibrationIni(filename)``
- :ref:`exhale_function_namespacecamera__calibration__parsers_1a7642240533c0a270f72baf3272d693a6` — ``parseCalibrationIni``

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

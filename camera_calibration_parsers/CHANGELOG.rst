^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package camera_calibration_parsers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

5.1.7 (2025-06-12)
------------------

5.1.6 (2025-04-02)
------------------

5.1.5 (2024-11-26)
------------------

5.1.4 (2024-06-27)
------------------

5.1.3 (2024-05-24)
------------------

5.1.2 (2024-04-16)
------------------
* Update to yaml-cpp 0.8.0. (`#305 <https://github.com/ros-perception/image_common/issues/305>`_)
* Contributors: Chris Lalancette

5.1.1 (2024-03-26)
------------------
* Switch from rcpputils::fs to std::filesystem (`#300 <https://github.com/ros-perception/image_common/issues/300>`_)
* Contributors: Christophe Bedard

5.1.0 (2024-01-24)
------------------

5.0.0 (2023-11-06)
------------------
* Removed C headers: camera_info_manager camera_calibration_parsers (`#290 <https://github.com/ros-perception/image_common/issues/290>`_)
* Contributors: Alejandro Hernández Cordero

4.5.1 (2023-09-07)
------------------

4.5.0 (2023-08-21)
------------------

4.3.0 (2023-04-28)
------------------

4.2.0 (2023-02-14)
------------------
* Update image_common to C++17. (`#267 <https://github.com/ros-perception/image_common/issues/267>`_)
* Contributors: Chris Lalancette

4.1.1 (2022-11-16)
------------------
* Add alias library targets for all libraries (`#259 <https://github.com/ros-perception/image_common/issues/259>`_)
* Contributors: RFRIEDM-Trimble

4.1.0 (2022-11-02)
------------------
* Add support for missing ROI and binning fields (`#254 <https://github.com/ros-perception/image_common/issues/254>`_)
* Contributors: AndreasR30

4.0.0 (2022-08-11)
------------------

3.2.1 (2022-07-12)
------------------

3.2.0 (2022-05-10)
------------------

3.1.4 (2022-03-25)
------------------
* Tests depend on rcpputils (`#236 <https://github.com/ros-perception/image_common/issues/236>`_)
* Contributors: Shane Loretz

3.1.3 (2022-03-01)
------------------
* Remove YAML_CPP_DLL define (`#231 <https://github.com/ros-perception/image_common/issues/231>`_)
* Contributors: Akash

3.1.2 (2022-01-14)
------------------
* Export a modern CMake target instead of variables and install includes to include/${PROJECT_NAME} (`#218 <https://github.com/ros-perception/image_common/issues/218>`_)
* Contributors: Shane Loretz

3.1.0 (2021-06-29)
------------------

3.0.0 (2021-05-26)
------------------
* Update maintainers (`#173 <https://github.com/ros-perception/image_common/issues/173>`_)
* Contributors: Alejandro Hernández Cordero

2.3.0 (2020-05-27)
------------------
* Fix formatting and include paths for linters (`#157 <https://github.com/ros-perception/image_common/issues/157>`_)
* Contributors: Martin Idel

2.2.1 (2019-10-23)
------------------
* ROS2 Using the filesystem helper in rcpputils (`#133 <https://github.com/ros-perception/image_common/issues/133>`_)
* [Windows][ros2] Avoid build break for Visual Studio 2019 v16.3 (`#135 <https://github.com/ros-perception/image_common/issues/135>`_)
* Contributors: Andreas Klintberg, Sean Yen

2.2.0 (2019-09-27)
------------------

2.1.1 (2019-05-30)
------------------

2.1.0 (2019-05-09)
------------------

2.0.0 (2018-12-05)
------------------
* Camera Calibration Parsers ROS2 Port (`#105 <https://github.com/ros-perception/image_common/issues/105>`_)
* Image Transport ROS2 port (`#84 <https://github.com/ros-perception/image_common/issues/84>`_)
* Contributors: Michael Carroll

1.11.13 (2017-11-05)
--------------------
* Use Boost_LIBRARIES instead of Boost_PYTHON_LIBRARY
  This was causing issues when building with python3 since then
  `Boost_PYTHON_LIBRARY` is not set, instead cmake sets
  `Boost_PYTHON3_LIBRARY`. So instead of adding each library separately,
  using `Boost_LIBRARIES` seems to be better. For reference, from the
  cmake docs:
  ```
  Boost_LIBRARIES        - Boost component libraries to be linked
  Boost\_<C>_LIBRARY      - Libraries to link for component <C>
  ```
* Contributors: Kartik Mohta, Vincent Rabaud

1.11.12 (2017-01-29)
--------------------
* Properly detect Boost Python 2 or 3
  This fixes `#59 <https://github.com/ros-perception/image_common/issues/59>`_
* 1.11.11
* update changelogs
* Contributors: Vincent Rabaud

1.11.11 (2016-09-24)
--------------------

1.11.10 (2016-01-19)
--------------------
* Add install target for python wrapper library
* Only link against needed Boost libraries
  9829b02 introduced a python dependency into find_package(Boost..) which
  results in ${Boost_LIBRARIES} containing boost_python and such a
  dependency to libpython at link time. With this patch we only link
  against the needed libraries.
* Contributors: Jochen Sprickerhof, Vincent Rabaud

1.11.9 (2016-01-17)
-------------------
* Add python wrapper for readCalibration.
  Reads .ini or .yaml calibration file and returns camera name and sensor_msgs/cameraInfo.
* Use $catkin_EXPORTED_TARGETS
* Contributors: Jochen Sprickerhof, Markus Roth

1.11.8 (2015-11-29)
-------------------
* Remove no-longer-neccessary flags to allow OS X to use 0.3 and 0.5 of yaml-cpp.
* remove buggy CMake message
* Contributors: Helen Oleynikova, Vincent Rabaud

1.11.7 (2015-07-28)
-------------------
* fix `#39 <https://github.com/ros-perception/image_common/issues/39>`_
* make sure test does not fail
* Contributors: Vincent Rabaud

1.11.6 (2015-07-16)
-------------------
* [camera_calibration_parsers] Better error message when calib file can't be written
* add rosbash as a test dependency
* add a test dependency now that we have tests
* parse distortion of arbitraty length in INI
  This fixes `#33 <https://github.com/ros-perception/image_common/issues/33>`_
* add a test to parse INI calibration files with 5 or 8 D param
* Add yaml-cpp case for building on Android
* Contributors: Gary Servin, Isaac IY Saito, Vincent Rabaud

1.11.5 (2015-05-14)
-------------------
* Fix catkin_make failure (due to yaml-cpp deps) for mac os
* Contributors: Yifei Zhang

1.11.4 (2014-09-21)
-------------------
* fix bad yaml-cpp usage in certain conditions
  fixes `#24 <https://github.com/ros-perception/image_common/issues/24>`_
* Contributors: Vincent Rabaud

1.11.3 (2014-05-19)
-------------------

1.11.2 (2014-02-13  08:32:06 +0100)
-----------------------------------
* add a dependency on pkg-config to have it work on Indigo

1.11.1 (2014-01-26  02:32:06 +0100)
-----------------------------------
* fix YAML CPP 0.5.x compatibility

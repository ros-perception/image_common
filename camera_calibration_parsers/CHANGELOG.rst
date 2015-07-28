^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package camera_calibration_parsers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

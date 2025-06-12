^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package camera_info_manager_py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

5.1.7 (2025-06-12)
------------------
* Fix CameraInfo distortion coefficients and logger (`#360 <https://github.com/ros-perception/image_common/issues/360>`_) (`#362 <https://github.com/ros-perception/image_common/issues/362>`_)
* Contributors: mergify[bot]

5.1.6 (2025-04-02)
------------------

5.1.5 (2024-11-26)
------------------
* Add `camera_info_manager_py` (`#335 <https://github.com/ros-perception/image_common/issues/335>`_) (`#336 <https://github.com/ros-perception/image_common/issues/336>`_)
  (cherry picked from commit d09c82cfb5558d253de99317a4d7d5fb61867b03)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Contributors: mergify[bot]

1.9.0 (2012-09-07 13:03)
------------------------

1.0.0 (2024-05-16)
------------------
* Ros2 (`#2 <https://github.com/clearpathrobotics/camera_info_manager_py/issues/2>`_)
  * Run magic converter
  * Ament_python package
  * Fix some imports
  * Remove references to cpp camera info manager.
  Disable tests
  * Linting
  * Fully Remove old tests
  * Add lint tests
  * Final tests
  * Remove pep257 from depends
* Contributors: Michael Hosmar

0.3.1 (2021-11-18)
------------------
* changelog
* Contributors: José Mastrangelo

0.3.0 (2021-11-18)
------------------
* added CPR maintainer
* Release to Melodic and Noetic
* Contributors: Jack O'Quin, José Mastrangelo, Lucas Walter, Martin Pecka

0.2.3 (2014-05-13)
------------------
* Only use rostest when testing enabled, thanks to Lukas Bulwahn.
* Move repository to ros-perception.
* Contributors: Jack O'Quin, Lukas Bulwahn

0.2.2 (2013-07-25)
------------------
* Add namespace parameter to constructor, so a driver can handle multiple cameras. Enhancement thanks to Martin Llofriu.
* Make unit tests conditional on ``CATKIN_ENABLE_TESTING``.
* Release to Groovy and Hydro.
* Contributors: Jack O'Quin, mllofriu

0.2.1 (2013-04-14)
------------------
* Set null calibration even when URL invalid (#7).
* Release to Groovy and Hydro.
* Contributors: Jack O'Quin

0.2.0 (2013-03-28)
------------------
* Convert to catkin.
* Remove roslib dependency.
* Release to Groovy and Hydro.
* Contributors: Jack O'Quin

0.1.0 (2012-12-05)
------------------
* Initial Python camera_info_manager release to Fuerte.
* Contributors: Jack O'Quin

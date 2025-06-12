^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package camera_info_manager_py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.12 (2025-06-12)
-------------------
* Fix CameraInfo distortion coefficients and logger (`#360 <https://github.com/ros-perception/image_common/issues/360>`_) (`#363 <https://github.com/ros-perception/image_common/issues/363>`_)
* Contributors: mergify[bot]

3.1.11 (2025-02-12)
-------------------

3.1.10 (2024-11-26)
-------------------
* Add `camera_info_manager_py` (backport `#335 <https://github.com/ros-perception/image_common/issues/335>`_) (`#337 <https://github.com/ros-perception/image_common/issues/337>`_)
  * Add `camera_info_manager_py` (`#335 <https://github.com/ros-perception/image_common/issues/335>`_)
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

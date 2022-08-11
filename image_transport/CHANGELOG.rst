^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package image_transport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.0.0 (2022-08-11)
------------------
* Remove subscriber and publisher impl methods without options (`#252 <https://github.com/ros-perception/image_common/issues/252>`_)
* Deprecate impl without options (`#249 <https://github.com/ros-perception/image_common/issues/249>`_)
* Contributors: Kenji Brameld

3.2.1 (2022-07-12)
------------------
* opt-in to qos overriding for publisher (`#246 <https://github.com/ros-perception/image_common/issues/246>`_)
* Contributors: Brian

3.2.0 (2022-05-10)
------------------
* Add qos option to override qos (`#208 <https://github.com/ros-perception/image_common/issues/208>`_)
* Contributors: Daisuke Nishimatsu

3.1.4 (2022-03-25)
------------------
* Image transport publisher crash fixes (`#235 <https://github.com/ros-perception/image_common/issues/235>`_)
* Contributors: RoboTech Vision

3.1.3 (2022-03-01)
------------------
* Simple IT plugins shutdown (`#225 <https://github.com/ros-perception/image_common/issues/225>`_)
* Remove PLUGINLIB__DISABLE_BOOST_FUNCTIONS definition. (`#226 <https://github.com/ros-perception/image_common/issues/226>`_)
* Contributors: Chris Lalancette, RoboTech Vision

3.1.2 (2022-01-14)
------------------
* Fix include order for cpplint (`#221 <https://github.com/ros-perception/image_common/issues/221>`_)
  Relates to https://github.com/ament/ament_lint/pull/324
* Export a modern CMake target instead of variables and install includes to include/${PROJECT_NAME} (`#218 <https://github.com/ros-perception/image_common/issues/218>`_)
* Contributors: Jacob Perron, Shane Loretz

3.1.0 (2021-06-29)
------------------
* Fix SimpleSubscriberPlugin (`#195 <https://github.com/ros-perception/image_common/issues/195>`_)
* Contributors: Ivan Santiago Paunovic

3.0.0 (2021-05-26)
------------------
* Make sure to mark overridden methods as 'override'. (`#192 <https://github.com/ros-perception/image_common/issues/192>`_)
* Expose subscription options (`#186 <https://github.com/ros-perception/image_common/issues/186>`_)
* fix mistyping 'cammera_publisher.hpp -> camera_publisher.hpp' (`#177 <https://github.com/ros-perception/image_common/issues/177>`_)
* Update maintainers (`#173 <https://github.com/ros-perception/image_common/issues/173>`_)
* make CameraPublisher::getNumSubscribers() work (`#163 <https://github.com/ros-perception/image_common/issues/163>`_)
* Contributors: Alejandro Hernández Cordero, Audrow Nash, Chris Lalancette, Hye-Jong KIM, Michael Ferguson

2.3.0 (2020-05-27)
------------------
* Fix formatting and include paths for linters (`#157 <https://github.com/ros-perception/image_common/issues/157>`_)
* Fix QoS initialization from RMW QoS profile (`#158 <https://github.com/ros-perception/image_common/issues/158>`_)
* Contributors: Jacob Perron, Martin Idel

2.2.1 (2019-10-23)
------------------
* add missing set header (`#140 <https://github.com/ros-perception/image_common/issues/140>`_)
* Contributors: Mikael Arguedas

2.2.0 (2019-09-27)
------------------

2.1.1 (2019-05-30)
------------------
* Update to use new count APIs (`#128 <https://github.com/ros-perception/image_common/issues/128>`_)
* use latest ros2 API (`#127 <https://github.com/ros-perception/image_common/issues/127>`_)
* Contributors: Karsten Knese, Michael Carroll

2.1.0 (2019-05-09)
------------------
* Update ROS2 branch to account for new NodeOptions interface (`#120 <https://github.com/ros-perception/image_common/issues/120>`_)
* Contributors: Carl Delsey

2.0.0 (2018-12-05)
------------------
* camera_info_manager ROS2 port (`#94 <https://github.com/ros-perception/image_common/issues/94>`_)
* Pointer api updates (`#104 <https://github.com/ros-perception/image_common/issues/104>`_)
* Fix rcutils API change by just removing it. (`#103 <https://github.com/ros-perception/image_common/issues/103>`_)
* [ROS2] corrections to remapping for raw images (`#97 <https://github.com/ros-perception/image_common/issues/97>`_)
* Make ROS2 ImageTransport conform to old api (`#88 <https://github.com/ros-perception/image_common/issues/88>`_)
* Image Transport ROS2 Port (`#84 <https://github.com/ros-perception/image_common/issues/84>`_)
* Contributors: Michael Carroll

1.11.13 (2017-11-05)
--------------------
* Disable image publisher plugins by name (`#60 <https://github.com/ros-perception/image_common/issues/60>`_)
  * Disable publisher plugins by name
  * Now have per publisher blacklist instead of image_transport wide.
* update to use non deprecated pluginlib macro
* Extend documentation of `getCameraInfoTopic`
  Document the fact that the `base_topic` argument must be resolved in order to build the correct camera info topic.
* Added cv::waitkey(10) for blank popup
  Without the cv::waitkey(10), it results in a blank popup which crashes/ leads to a black popup. This change corrects that problem.
  ROS Kinetic, Ubuntu 16.04.3
* Contributors: Aaditya Saraiya, Lucas Walter, Mikael Arguedas, Thibaud Chupin, Vincent Rabaud

1.11.12 (2017-01-29)
--------------------
* Fix CMake of image_transport/tutorial and polled_camera
  Fix loads of problems with the CMakeLists.
* image_transport/tutorial: Add dependency on generated msg
  Without this, build fails on Kinetic because ResizedImage.h has not been
  generated yet.
* image_transport/tutorial: Add missing catkin_INCLUDE_DIRS
  Without this, compilation files on Kinetic because ros.h cannot be found.
* 1.11.11
* update changelogs
* Contributors: Martin Guenther, Vincent Rabaud

1.11.11 (2016-09-24)
--------------------

1.11.10 (2016-01-19)
--------------------

1.11.9 (2016-01-17)
-------------------
* fix linkage in tutorials
* Use $catkin_EXPORTED_TARGETS
* Contributors: Jochen Sprickerhof, Vincent Rabaud

1.11.8 (2015-11-29)
-------------------

1.11.7 (2015-07-28)
-------------------

1.11.6 (2015-07-16)
-------------------

1.11.5 (2015-05-14)
-------------------
* image_transport: fix CameraSubscriber shutdown (circular shared_ptr ref)
  CameraSubscriber uses a private boost::shared_ptr to share an impl object
  between copied instances. In CameraSubscriber::CameraSubscriber(), it
  handed this shared_ptr to boost::bind() and saved the created wall timer
  in the impl object, thus creating a circular reference. The impl object
  was therefore never freed.
  Fix that by passing a plain pointer to boost::bind().
* avoid a memory copy for the raw publisher
* add a way to publish an image with only the data pointer
* Make function inline to avoid duplicated names when linking statically
* add plugin examples for the tutorial
* update instructions for catkin
* remove uselessly linked library
  fixes `#28 <https://github.com/ros-perception/image_common/issues/28>`_
* add a tutorial for image_transport
* Contributors: Gary Servin, Max Schwarz, Vincent Rabaud

1.11.4 (2014-09-21)
-------------------

1.11.3 (2014-05-19)
-------------------

1.11.2 (2014-02-13)
-------------------

1.11.1 (2014-01-26 02:33)
-------------------------

1.11.0 (2013-07-20 12:23)
-------------------------

1.10.5 (2014-01-26 02:34)
-------------------------

1.10.4 (2013-07-20 11:42)
-------------------------
* add Jack as maintainer
* update my email address
* Contributors: Vincent Rabaud

1.10.3 (2013-02-21 05:33)
-------------------------

1.10.2 (2013-02-21 04:48)
-------------------------

1.10.1 (2013-02-21 04:16)
-------------------------

1.10.0 (2013-01-13)
-------------------
* fix the urls
* use the pluginlib script to remove some warnings
* added license headers to various cpp and h files
* Contributors: Aaron Blasdel, Vincent Rabaud

1.9.22 (2012-12-16)
-------------------
* get rid of the deprecated class_loader interface
* Contributors: Vincent Rabaud

1.9.21 (2012-12-14)
-------------------
* CMakeLists.txt clean up
* Updated package.xml file(s) to handle new catkin buildtool_depend
  requirement
* Contributors: William Woodall, mirzashah

1.9.20 (2012-12-04)
-------------------

1.9.19 (2012-11-08)
-------------------
* add the right link libraries
* Contributors: Vincent Rabaud

1.9.18 (2012-11-06)
-------------------
* Isolated plugins into their own library to follow new
  class_loader/pluginlib guidelines.
* remove the brief attribute
* Contributors: Mirza Shah, Vincent Rabaud

1.9.17 (2012-10-30 19:32)
-------------------------

1.9.16 (2012-10-30 09:10)
-------------------------
* add xml file
* Contributors: Vincent Rabaud

1.9.15 (2012-10-13 08:43)
-------------------------
* fix bad folder/libraries
* Contributors: Vincent Rabaud

1.9.14 (2012-10-13 01:07)
-------------------------

1.9.13 (2012-10-06)
-------------------

1.9.12 (2012-10-04)
-------------------

1.9.11 (2012-10-02 02:56)
-------------------------

1.9.10 (2012-10-02 02:42)
-------------------------

1.9.9 (2012-10-01)
------------------
* fix dependencies
* Contributors: Vincent Rabaud

1.9.8 (2012-09-30)
------------------
* add catkin as a dependency
* comply to the catkin API
* Contributors: Vincent Rabaud

1.9.7 (2012-09-18 11:39)
------------------------

1.9.6 (2012-09-18 11:07)
------------------------

1.9.5 (2012-09-13)
------------------
* install the include directories
* Contributors: Vincent Rabaud

1.9.4 (2012-09-12 23:37)
------------------------

1.9.3 (2012-09-12 20:44)
------------------------

1.9.2 (2012-09-10)
------------------

1.9.1 (2012-09-07 15:33)
------------------------
* make the libraries public
* Contributors: Vincent Rabaud

1.9.0 (2012-09-07 13:03)
------------------------
* catkinize for Groovy
* Initial image_common stack check-in, containing image_transport.
* Contributors: Vincent Rabaud, gerkey, kwc, mihelich, pmihelich, straszheim, vrabaud

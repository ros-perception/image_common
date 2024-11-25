cmake_minimum_required(VERSION 3.8)
project(camera_info_manager_py)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

ament_export_dependencies(sensor_msgs)
ament_package()

catkin_install_python(
  PROGRAMS 
    src/camera_info_manager/camera_info_manager.py
    src/camera_info_manager/zoom_camera_info_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Unit test uses nose, but needs rostest to create a ROS environment.
if (BUILD_TESTING)
  find_package(roscpp REQUIRED)
  find_package(camera_info_manager REQUIRED)

  find_package(rostest REQUIRED)
  add_executable(generate_camera_info
    tests/generate_camera_info.cpp
    )
  # TODO(lucasw) Can the roscpp and camera_info_manager dependencies
  # get added to catkin_LIBRARIES somehow?
  target_link_libraries(generate_camera_info ${catkin_LIBRARIES}
      ${roscpp_LIBRARIES} ${camera_info_manager_LIBRARIES})

  add_rostest(tests/load_cpp_camera_info.test DEPENDENCIES generate_camera_info)
  add_rostest(tests/unit_test.test)
endif(CATKIN_ENABLE_TESTING)

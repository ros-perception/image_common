cmake_minimum_required(VERSION 3.5)

project(camera_info_manager)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Boost REQUIRED)
include_directories(include)

# add a library
add_library(${PROJECT_NAME} src/camera_info_manager.cpp)
ament_target_dependencies(${PROJECT_NAME}
  "ament_index_cpp"
  "rclcpp"
  "camera_calibration_parsers"
  "image_transport"
  "sensor_msgs"
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

if(ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # TODO(ros2): Reenable when a suitable rostest replacement is available.
  #find_package(rostest)
  # Unit test uses gtest, but needs rostest to create a ROS environment.
  # Hence, it must be created as a normal executable, not using
  # catkin_add_gtest() which runs an additional test without rostest.
  #add_executable(unit_test tests/unit_test.cpp)
  #target_link_libraries(unit_test ${PROJECT_NAME} ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

  #add_rostest(tests/unit_test.test DEPENDENCIES unit_test)
endif()

ament_package()

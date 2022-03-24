cmake_minimum_required(VERSION 3.5)

project(image_transport)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)

find_package(message_filters REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

# Build image_transport library
add_library(${PROJECT_NAME}
  src/camera_common.cpp
  src/publisher.cpp
  src/subscriber.cpp
  src/single_subscriber_publisher.cpp
  src/camera_publisher.cpp
  src/camera_subscriber.cpp
  src/image_transport.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  message_filters
  pluginlib
  rclcpp
  sensor_msgs
)

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(${PROJECT_NAME} PRIVATE "IMAGE_TRANSPORT_BUILDING_DLL")

# Build image_transport_plugins library (raw)
add_library(${PROJECT_NAME}_plugins
  src/manifest.cpp
)
target_link_libraries(${PROJECT_NAME}_plugins ${PROJECT_NAME})

# Build list_transports
add_executable(list_transports src/list_transports.cpp)
target_link_libraries(list_transports ${PROJECT_NAME})

# Build republish
add_executable(republish src/republish.cpp)
target_link_libraries(republish ${PROJECT_NAME})

# Install plugin descriptions
pluginlib_export_plugin_description_file(image_transport default_plugins.xml)

# Install libraries
install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(
  TARGETS list_transports republish
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(message_filters rclcpp sensor_msgs pluginlib)

if(BUILD_TESTING)
  #TODO(ros2) enable linting
  #find_package(ament_lint_auto REQUIRED)
  #ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)

  ament_add_gtest(${PROJECT_NAME}-camera_common test/test_camera_common.cpp)
  if(TARGET ${PROJECT_NAME}-camera_common)
    target_link_libraries(${PROJECT_NAME}-camera_common ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-publisher test/test_publisher.cpp)
  if(TARGET ${PROJECT_NAME}-publisher)
    target_link_libraries(${PROJECT_NAME}-publisher ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-subscriber test/test_subscriber.cpp)
  if(TARGET ${PROJECT_NAME}-subscriber)
    target_link_libraries(${PROJECT_NAME}-subscriber ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-message_passing test/test_message_passing.cpp)
  if(TARGET ${PROJECT_NAME}-message_passing)
    target_link_libraries(${PROJECT_NAME}-message_passing ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-remapping test/test_remapping.cpp)
  if(TARGET ${PROJECT_NAME}-remapping)
    target_link_libraries(${PROJECT_NAME}-remapping ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-single_subscriber_publisher test/test_single_subscriber_publisher.cpp)
  if(TARGET ${PROJECT_NAME}-single_subscriber_publisher)
    target_link_libraries(${PROJECT_NAME}-single_subscriber_publisher ${PROJECT_NAME})
  endif()
endif()

ament_package()

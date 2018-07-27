cmake_minimum_required(VERSION 3.5)

project(image_transport)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tinyxml_vendor REQUIRED)
find_package(TinyXML REQUIRED)

include_directories(include)

# Build libimage_transport
add_library(${PROJECT_NAME}
  src/camera_common.cpp
  src/publisher.cpp
  src/subscriber.cpp
  src/single_subscriber_publisher.cpp
  src/camera_publisher.cpp
  src/camera_subscriber.cpp
  src/image_transport.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  "message_filters"
  "pluginlib"
  "rclcpp"
  "sensor_msgs"
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

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

  ament_add_gtest(${PROJECT_NAME}-single_subscriber_publisher test/test_single_subscriber_publisher.cpp)
  if(TARGET ${PROJECT_NAME}-single_subscriber_publisher)
    target_link_libraries(${PROJECT_NAME}-single_subscriber_publisher ${PROJECT_NAME})
  endif()
endif()


#add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME}
#  ${Boost_LIBRARIES}
#  ${catkin_LIBRARIES}
#)

# Build libimage_transport_plugins
#add_library(${PROJECT_NAME}_plugins src/manifest.cpp src/raw_publisher.cpp)
#target_link_libraries(${PROJECT_NAME}_plugins ${PROJECT_NAME})
#
#install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugins
#        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        COMPONENT main
#)
#install(DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

# add two execs
#add_executable(republish src/republish.cpp)
#target_link_libraries(republish ${PROJECT_NAME})

#add_executable(list_transports src/list_transports.cpp)
#target_link_libraries(list_transports
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)

#install(TARGETS list_transports republish
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

# add xml file
#install(FILES default_plugins.xml
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

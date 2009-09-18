cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_boost_directories()
rosbuild_add_library(${PROJECT_NAME} src/publisher.cpp src/subscriber.cpp src/camera_subscriber.cpp src/raw_publisher.cpp src/raw_subscriber.cpp src/manifest.cpp)
rosbuild_link_boost(${PROJECT_NAME} system)

rosbuild_add_executable(transform src/transform.cpp)
target_link_libraries(transform ${PROJECT_NAME})

rosbuild_add_executable(list_transports src/list_transports.cpp)
target_link_libraries(list_transports ${PROJECT_NAME})

#rosbuild_add_executable(test_publisher src/test_publisher.cpp)
#target_link_libraries(test_publisher ${PROJECT_NAME})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

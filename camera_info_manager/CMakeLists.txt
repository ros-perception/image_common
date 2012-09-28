cmake_minimum_required(VERSION 2.8)
project(camera_info_manager)

find_package(catkin REQUIRED)
catkin_package(INCLUDE_DIRS include
               LIBRARIES camera_info_manager
)

find_package(catkin COMPONENTS camera_calibration_parsers image_transport roscpp roslib)

include_directories(SYSTEM ${camera_calibration_parsers_INCLUDE_DIRS}
                           ${image_transport_INCLUDE_DIRS}
                           ${roscpp_INCLUDE_DIRS}
)
include_directories(include)

# add a library
add_library(${PROJECT_NAME} src/camera_info_manager.cpp)
target_link_libraries(${PROJECT_NAME} ${camera_calibration_parsers_LIBRARIES}
                                      ${roscpp_LIBRARIES}
                                      ${roslib_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        COMPONENT main
)
install(DIRECTORY include/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

# unit test uses gtest, but needs rostest to create a ROS environment
catkin_add_gtest(unit_test tests/unit_test.cpp)
target_link_libraries(unit_test ${PROJECT_NAME})

if (FALSE)
rosbuild_add_rostest(tests/unit_test.test)
endif()

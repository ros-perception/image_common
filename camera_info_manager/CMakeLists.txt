project(camera_info_manager)
catkin_project(${PROJECT_NAME} INCLUDE_DIRS include)

find_package(ROS COMPONENTS roscpp image_transport camera_calibration_parsers image_transport)

include_directories(SYSTEM ${roscpp_INCLUDE_DIRS}
                           ${image_transport_INCLUDE_DIRS}
                           ${camera_calibration_parsers_INCLUDE_DIRS}
)
include_directories(include)

add_library(${PROJECT_NAME} src/camera_info_manager.cpp)

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib
        COMPONENT main
)

# unit test uses gtest, but needs rostest to create a ROS environment
return()
catkin_add_gtest(unit_test tests/unit_test.cpp)
target_link_libraries(unit_test ${PROJECT_NAME})

rosbuild_add_rostest(tests/unit_test.test)

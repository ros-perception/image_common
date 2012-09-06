add_library(camera_info_manager src/camera_info_manager.cpp)

# unit test uses gtest, but needs rostest to create a ROS environment
return()
catkin_add_gtest(unit_test tests/unit_test.cpp)
target_link_libraries(unit_test camera_info_manager)

rosbuild_add_rostest(tests/unit_test.test)

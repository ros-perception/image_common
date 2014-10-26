cmake_minimum_required(VERSION 2.8)
project(image_transport_tutorial)

find_package(catkin REQUIRED cv_bridge image_transport)

catkin_package()

add_executable(my_publisher src/my_publisher.cpp)
target_link_libraries(my_publisher ${catkin_LIBRARIES})

add_executable(my_subscriber src/my_subscriber.cpp)
target_link_libraries(my_subscriber ${catkin_LIBRARIES})

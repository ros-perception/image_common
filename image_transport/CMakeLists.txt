project(image_transport)
catkin_project(${PROJECT_NAME} INCLUDE_DIRS include)

find_package(Boost)
#find_package(ROS COMPONENTS pluginlib roscpp)
#TODO, use the above for Groovy
find_package(ROS COMPONENTS roscpp)

include_directories(SYSTEM ${pluginlib_INCLUDE_DIRS} ${roscpp_INCLUDE_DIRS})
include_directories(include)

#TODO remove that uglyniess for Groovy
include_directories(/opt/ros/fuerte/stacks/pluginlib/include)





# add a library
add_library(${PROJECT_NAME} src/publisher.cpp src/subscriber.cpp src/single_subscriber_publisher.cpp src/image_transport.cpp src/camera_common.cpp src/camera_publisher.cpp src/camera_subscriber.cpp src/manifest.cpp)
target_link_libraries(${PROJECT_NAME} ${BOOST_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib
        COMPONENT main
)

# add two execs
add_executable(republish src/republish.cpp)
target_link_libraries(republish ${PROJECT_NAME})

add_executable(list_transports src/list_transports.cpp)
target_link_libraries(list_transports ${PROJECT_NAME})

install(TARGETS list_transports republish
        DESTINATION share/${PROJECT_NAME}/bin
)

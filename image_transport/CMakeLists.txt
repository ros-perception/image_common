cmake_minimum_required(VERSION 2.8)
project(image_transport)

find_package(catkin REQUIRED)
catkin_package(INCLUDE_DIRS include
               LIBRARIES image_transport
)

find_package(Boost)
find_package(catkin COMPONENTS message_filters pluginlib rosconsole roscpp roslib)

include_directories(SYSTEM ${pluginlib_INCLUDE_DIRS} ${roscpp_INCLUDE_DIRS})
include_directories(include)

# add a library
add_library(${PROJECT_NAME} src/publisher.cpp src/subscriber.cpp src/single_subscriber_publisher.cpp src/image_transport.cpp src/camera_common.cpp src/camera_publisher.cpp src/camera_subscriber.cpp src/manifest.cpp)
target_link_libraries(${PROJECT_NAME} ${BOOST_LIBRARIES}
                                      ${message_filters_LIBRARIES}
                                      ${pluginlib_LIBRARIES}
                                      ${rosconsole_LIBRARIES}
                                      ${roscpp_LIBRARIES}
                                      ${roslib_LIBRARIES}
                                      tinyxml
)

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        COMPONENT main
)
install(DIRECTORY include/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

# add two execs
add_executable(republish src/republish.cpp)
target_link_libraries(republish ${PROJECT_NAME})

add_executable(list_transports src/list_transports.cpp)
target_link_libraries(list_transports ${PROJECT_NAME}
                                      ${message_filters_LIBRARIES}
                                      ${roscpp_LIBRARIES}
                                      ${roslib_LIBRARIES}
)

install(TARGETS list_transports republish
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

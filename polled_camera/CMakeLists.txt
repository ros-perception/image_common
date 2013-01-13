cmake_minimum_required(VERSION 2.8)
project(polled_camera)

# generate the server
find_package(catkin REQUIRED message_generation roscpp sensor_msgs std_msgs)

add_service_files(DIRECTORY srv FILES GetPolledImage.srv)

generate_messages(DEPENDENCIES sensor_msgs std_msgs)

# define the project
catkin_package(DEPENDS roscpp sensor_msgs
               INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
)


# create some library and exe
find_package(catkin REQUIRED image_transport rosconsole roscpp)
include_directories(SYSTEM ${image_transport_INCLUDE_DIRS})
include_directories(include
                    ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

add_library(${PROJECT_NAME} src/publication_server.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_executable(poller src/poller.cpp)
target_link_libraries(poller ${PROJECT_NAME}
                             ${catkin_LIBRARIES}
)
install(TARGETS poller
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

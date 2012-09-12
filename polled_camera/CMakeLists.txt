project(polled_camera)
catkin_project(${PROJECT_NAME} INCLUDE_DIRS include)

# generate the server
find_package(catkin REQUIRED genmsg sensor_msgs std_msgs)

add_service_files(DIRECTORY srv FILES GetPolledImage.srv)

generate_messages(DEPENDENCIES sensor_msgs std_msgs)


# create some library and exe
find_package(catkin REQUIRED image_transport rosconsole roscpp)
include_directories(SYSTEM ${image_transport_INCLUDE_DIRS})
include_directories(include
                    ${CATKIN_BUILD_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

add_library(${PROJECT_NAME} src/publication_server.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME} ${rosconsole_LIBRARIES})
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PROJECT_LIB_DESTINATION}
)
install(DIRECTORY include/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

add_executable(poller src/poller.cpp)
target_link_libraries(poller ${PROJECT_NAME}
                             ${roscpp_LIBRARIES}
)
install(TARGETS poller
        DESTINATION ${CATKIN_PROJECT_BIN_DESTINATION}
)

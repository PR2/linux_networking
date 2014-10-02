cmake_minimum_required(VERSION 2.8.3)
project(network_detector)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
add_executable(network_detector src/network_detector.cpp)
target_link_libraries(network_detector ${CATKIN_LIBRARIES})

catkin_package(
    DEPENDS roscpp std_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(TARGETS network_detector 
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} 
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

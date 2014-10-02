# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(wpa_supplicant_node)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs dynamic_reconfigure)


include_directories(wpa_supplicant/wpa_supplicant ${catkin_INCLUDE_DIRS})
generate_dynamic_reconfigure_options(cfg/WpaSupplicantNode.cfg)

add_executable(wpa_supplicant_node src/nodes/wpa_supplicant_node.cpp)
target_link_libraries(wpa_supplicant_node ${WPA_SUPPLICANT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(wpa_supplicant_node wpa_supplicant_node_gencpp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_node_gencfg ${PROJECT_NAME}_INCLUDE_DIRS)

add_message_files(
    FILES 
    Network.msg
    FrequencyList.msg
    NetworkList.msg
    Bss.msg
    NetworkParameter.msg
    SecurityProperties.msg
)
add_service_files(
    FILES 
    AddNetwork.srv
    RemoveNetwork.srv
    SetNetworkState.srv
    SetNetworkParameters.srv
)
add_action_files(
    FILES 
    Scan.action
    Associate.action
)


generate_messages(
    DEPENDENCIES roscpp actionlib actionlib_msgs dynamic_reconfigure
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roscpp actionlib actionlib_msgs dynamic_reconfigure 
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS wpa_supplicant# TODO include
    LIBRARIES # TODO
)

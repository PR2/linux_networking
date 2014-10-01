# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(wpa_supplicant_node)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs dynamic_reconfigure libpcsclite-dev libncurses-dev libnl-dev libreadline-dev libdbus-dev)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include(FindPkgConfig)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

# Snippet for generating actions
rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

# Find the build flags for libwpa_supplicant
set(ENV{PKG_CONFIG_PATH} "${PROJECT_SOURCE_DIR}/wpa_supplicant/wpa_supplicant:$ENV{PKG_CONFIG_PATH}")
pkg_check_modules(WPA_SUPPLICANT REQUIRED libwpa_supplicant)

link_directories(${WPA_SUPPLICANT_LIBRARY_DIRS})
include_directories(${WPA_SUPPLICANT_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()
rosbuild_gensrv()

#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

# add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

add_executable(wpa_supplicant_node src/nodes/wpa_supplicant_node.cpp)
target_link_libraries(wpa_supplicant_node ${WPA_SUPPLICANT_LIBRARIES})

# CATKIN_MIGRATION
# use link_directories() include_directories(), add_definitions(), target_link_libraries() or set_target_properties
# rosbuild_add_link_flags(wpa_supplicant_node ${WPA_SUPPLICANT_LDFLAGS_OTHER})

# CATKIN_MIGRATION
# use link_directories() include_directories(), add_definitions(), target_link_libraries() or set_target_properties
# rosbuild_add_compile_flags(wpa_supplicant_node ${WPA_SUPPLICANT_CFLAGS_OTHER})
## Generate added messages and services with any dependencies listed here
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
    entries
    all-wcprops
    SetNetworkParameters.srv.svn-base
    SetNetworkState.srv.svn-base
    RemoveNetwork.srv.svn-base
    AddNetwork.srv.svn-base
)
add_action_files(
    FILES 
    Scan.action
    Associate.action
    dir-prop-base
    entries
    all-wcprops
    Scan.action.svn-base
    Associate.action.svn-base
)

#generate_messages(
#    #TODO DEPENDENCIES roscpp actionlib actionlib_msgs dynamic_reconfigure libpcsclite-dev libncurses-dev libnl-dev libreadline-dev libdbus-dev
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roscpp actionlib actionlib_msgs dynamic_reconfigure libpcsclite-dev libncurses-dev libnl-dev libreadline-dev libdbus-dev
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

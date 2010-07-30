cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
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

rosbuild_init()

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

#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

rosbuild_add_executable(wpa_supplicant_node src/nodes/wpa_supplicant_node.cpp)
target_link_libraries(wpa_supplicant_node ${WPA_SUPPLICANT_LIBRARIES})
rosbuild_add_link_flags(wpa_supplicant_node ${WPA_SUPPLICANT_LDFLAGS_OTHER})
rosbuild_add_compile_flags(wpa_supplicant_node ${WPA_SUPPLICANT_CFLAGS_OTHER})

# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(asmach)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED )

catkin_package(
    DEPENDS # TODO add dependencies
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)


install(PROGRAMS /src/asmach/concurrence.py /src/asmach/state_machine.py /src/asmach/tb_edit.py /src/asmach/log.py /src/asmach/async.py /src/asmach/util.py /src/asmach/iterator.py /src/asmach/state.py /src/asmach/user_data.py /src/asmach/sequence.py /src/asmach/exceptions.py /src/asmach/container.py /src/asmach/__init__.py  USE_SOURCE_PERMISSIONS 
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY src asmach  
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})




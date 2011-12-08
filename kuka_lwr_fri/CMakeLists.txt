#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#project(kuka_lwr_fri)

#
# Do setup in case of ros package, If ROS_ROOT is set, it is
# recommended to use RTT/OCL through the ros packages.
#
set (ROS_ROOT $ENV{ROS_ROOT} )
if (ROS_ROOT)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()
  rosbuild_find_ros_package( rtt )
  set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/../install )
endif()

# Set the CMAKE_PREFIX_PATH in case you're not using Orocos through ROS
# for helping these find commands find RTT.
find_package(Orocos-RTT REQUIRED ${RTT_HINTS})

if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/friComm.h)
  message(FATAL_ERROR "
   You NEED TO copy the friComm.h file that you got from KUKA in this directory:
   ${CMAKE_CURRENT_SOURCE_DIR}

   Otherwise, this will never compile !
   ")
endif()

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

orocos_typegen_headers(--notransports=corba,typelib,mqueue friComm.h)

#
# Provided 'uninstall' target
#
orocos_generate_package()

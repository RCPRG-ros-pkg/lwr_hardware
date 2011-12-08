cmake_minimum_required(VERSION 2.4.6)

#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
# The CMake 'target' names are identical to the first argument of the
# macro, except for orocos_typekit, where the target is fully
# controlled by generated code of 'typegen'.
#
project(lwr_fri)

#
# Do setup in case of ros package, If ROS_ROOT is set, it is
# recommended to use RTT/OCL through the ros packages.
#
set (ROS_ROOT $ENV{ROS_ROOT} )
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_find_ros_package( rtt )
set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/../install )

rosbuild_genmsg()
rosbuild_include(rtt_rosnode GenerateRTTtypekit )
ros_generate_rtt_typekit(lwr_fri)

# Set the CMAKE_PREFIX_PATH in case you're not using Orocos through ROS
# for helping these find commands find RTT.
find_package(Orocos-RTT REQUIRED ${RTT_HINTS})

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

#common commands for building c++ executables and libraries
find_path(RTNET_INCLUDE_DIR NAMES rtnet.h PATH_SUFFIXES rtnet/include)
if(RTNET_INCLUDE_DIR AND ${OROCOS_TARGET} STREQUAL "xenomai" )
  message("Found RTNET: ${RTNET_INCLUDE_DIR}")
  include_directories(${RTNET_INCLUDE_DIR})
  add_definitions(-DHAVE_RTNET)
  # try Xenomai
  execute_process(COMMAND xeno-config --skin=posix --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND xeno-config --skin=posix --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${XENO_CFLAGS}")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS}")
  orocos_component(lwr_fri src/FRIComponent.cpp) # ...you may add multiple source files
  orocos_add_compile_flags(lwr_fri ${XENO_CFLAGS})
  orocos_add_link_flags(lwr_fri ${XENO_LDFLAGS})
else()
  orocos_component(lwr_fri src/FRIComponent.cpp) # ...you may add multiple source files
  orocos_component(lwr_diagnostics src/FRIDiagnostics.cpp)
endif()

# Link with the ROS .msg typekit of this package
# rtt-lwr_fri-typekit is the cmake target defined by ros_generate_rtt_typekit() above
target_link_libraries(lwr_fri rtt-lwr_fri-typekit )

# Generates the typekit which defines float32[7]
orocos_typekit(lwr_fri_typekit lwr_fri-typekit.cpp)

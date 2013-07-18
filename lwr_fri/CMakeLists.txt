cmake_minimum_required(VERSION 2.8.3)
project(lwr_fri)

find_package(catkin REQUIRED COMPONENTS rtt_ros kuka_lwr_fri diagnostic_msgs geometry_msgs std_msgs message_generation kdl_conversions)

add_message_files(DIRECTORY msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  DEPENDS rtt
  CATKIN_DEPENDS rtt_ros diagnostic_msgs geometry_msgs std_msgs message_runtime
)

find_package(OROCOS-RTT REQUIRED)
find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
add_definitions(-DRTT_COMPONENT)
orocos_component(fricomponent src/FRIComponent.cpp)
target_link_libraries(fricomponent ${orocos_kdl_LIBRARIES} ${kdl_conversions_LIBRARIES})
add_dependencies(fricomponent ${PROJECT_NAME}_gencpp)

orocos_component(fridiagnostics src/FRIDiagnostics.cpp)
add_dependencies(fridiagnostics ${PROJECT_NAME}_gencpp)

if(${OROCOS_TARGET} STREQUAL "xenomai" )
# message("Found RTNET: ${RTNET_INCLUDE_DIR}")
# include_directories(${RTNET_INCLUDE_DIR})
  add_definitions(-DHAVE_RTNET)
  # try Xenomai
  execute_process(COMMAND xeno-config --skin=posix --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND xeno-config --skin=posix --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${XENO_CFLAGS}" -DXENOMAI)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS}")
#  orocos_component(lwrfri src/FRIComponent.cpp) # ...you may add multiple source files
  orocos_add_compile_flags(fricomponent ${XENO_CFLAGS})
  orocos_add_link_flags(fricomponent ${XENO_LDFLAGS})
  TARGET_LINK_LIBRARIES(fricomponent "-lrtdm")
endif()

orocos_generate_package()


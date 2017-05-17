cmake_minimum_required(VERSION 2.8.3)
project(lwr_fri)

find_package(catkin REQUIRED COMPONENTS rtt_ros kuka_lwr_fri diagnostic_msgs geometry_msgs std_msgs lwr_msgs message_generation kdl_conversions cmake_modules)

add_message_files(DIRECTORY msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  DEPENDS rtt
  CATKIN_DEPENDS rtt_ros diagnostic_msgs geometry_msgs std_msgs lwr_msgs message_runtime
)

find_package(OROCOS-RTT REQUIRED)
find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

if(${OROCOS_TARGET} STREQUAL "xenomai" )
  find_package(Xenomai REQUIRED)
  add_definitions(-DHAVE_RTNET)
  add_definitions(-DXENOMAI)
endif()

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

orocos_component(fricomponent src/FRIComponent.cpp)
if(${OROCOS_TARGET} STREQUAL "xenomai" )
  target_link_libraries(fricomponent ${orocos_kdl_LIBRARIES} ${kdl_conversions_LIBRARIES} ${Xenomai_RTDM_LIBRARIES})
else()
  target_link_libraries(fricomponent ${orocos_kdl_LIBRARIES} ${kdl_conversions_LIBRARIES})
endif()
add_dependencies(fricomponent ${PROJECT_NAME}_gencpp)

orocos_component(fridiagnostics src/FRIDiagnostics.cpp)
add_dependencies(fridiagnostics ${PROJECT_NAME}_gencpp)


orocos_generate_package()


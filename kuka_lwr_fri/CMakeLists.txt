cmake_minimum_required(VERSION 2.8.3)
project(kuka_lwr_fri)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include/kuka_lwr_fri/friComm.h)
  message(FATAL_ERROR "
   You NEED TO copy the friComm.h file that you got from KUKA in this directory:
   ${CMAKE_CURRENT_SOURCE_DIR}

   Otherwise, this will never compile !
   ")
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

#install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)


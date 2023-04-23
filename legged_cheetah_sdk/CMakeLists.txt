cmake_minimum_required(VERSION 3.0.2)
project(legged_cheetah_sdk)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES legged_cheetah_sdk
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/IMU/mip.c
  src/IMU/mip_sdk_3dm.c
  src/IMU/mip_sdk_ahrs.c
  src/IMU/mip_sdk_base.c
  src/IMU/mip_sdk_filter.c
  src/IMU/mip_sdk_gps.c
  src/IMU/mip_sdk_inteface.c
  src/IMU/mip_sdk_system.c
  src/IMU/ring_buffer.c
  src/IMU/mip_sdk_user_functions.c
  src/IMU/byteswap_utilities.c
  src/IMU/LordImu.cpp
)

add_subdirectory(test)


## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   private lord_imu 
# )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


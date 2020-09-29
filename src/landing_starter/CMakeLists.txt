cmake_minimum_required(VERSION 2.8.3)
project(landing_starter)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  dji_sdk
  image_transport
  move_base
  tf
  sensor_msgs
)

find_package(DJIOSDK REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_drone
#  CATKIN_DEPENDS guidance move_base
#  DEPENDS system_lib
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


add_executable(landing_starter
        src/landing_starter.cpp)
target_link_libraries(landing_starter
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(landing_starter dji_sdk_generate_messages_cpp)



install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        )



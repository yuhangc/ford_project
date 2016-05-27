cmake_minimum_required(VERSION 2.8.3)
project(ford_project)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  haptic_msg.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ford_project
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
	CATKIN_DEPENDS message_runtime
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include/ford_project
    external/Arduino
    external/Sensoray826
    ${catkin_INCLUDE_DIRS})

link_directories(external/Sensoray826)

# set source code and headers
set(IMU_MANAGER_SOURCES
    src/imu_manager.cpp
    external/Arduino/CArduinoDevice.cpp
)

set(IMU_MANAGER_HEADERS
    include/ford_project/imu_manager.h
    external/Arduino/CArduinoDevice.h
)

set(HAPTIC_SOURCES
    src/haptic_test.cpp
)

## Declare a C++ executable
add_executable(imu_manager ${IMU_MANAGER_SOURCES} ${IMU_MANAGER_HEADERS})
target_link_libraries(imu_manager ${catkin_LIBRARIES})

add_executable(haptic_test ${HAPTIC_SOURCES})
target_link_libraries(haptic_test ${catkin_LIBRARIES} 826_64)
## Add cmake target dependencies of the executable
# add_dependencies(ford_project_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

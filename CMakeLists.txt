cmake_minimum_required(VERSION 2.8.3)
project(ford_project)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs

  message_generation
  cmake_modules
  cv_bridge
)

find_package(Eigen REQUIRED)

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
  CATKIN_DEPENDS message_runtime
  DEPENDS Eigen
)


###########
## Build ##
###########

## specify qt packages and directories
find_package(Qt5Widgets REQUIRED)
include_directories(${Qt5Widgets_INCLUDES})

## specify the aruco package and directories
find_package(aruco REQUIRED)
include_directories(${aruco_INCLUDE_DIRS})

## specify openCV package and directories
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include/ford_project
    external/Arduino
    external/Sensoray826
    external/MC-USB3101
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

link_directories(external/Sensoray826)
link_directories(external/MC-USB3101)
link_directories(${aruco_LIB_DIR})

# set c++ compiler options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# set sources code and headers for IMU manager
set(IMU_MANAGER_SOURCES
    src/imu_manager.cpp
    external/Arduino/CArduinoDevice.cpp
)

set(IMU_MANAGER_HEADERS
    include/ford_project/imu_manager.h
    external/Arduino/CArduinoDevice.h
)

# set sources and headers for haptic thread
set(HAPTIC_PCI_SOURCES
    src/haptic_test_PCI.cpp
)

set(HAPTIC_PCI_HEADERS
    include/ford_project/haptic_test_PCI.h
)

set(HAPTIC_USB_SOURCES
    src/haptic_USB.cpp
)

set(HAPTIC_USB_HEADERS
    include/ford_project/haptic_USB.h
)

# set sources and headers for gesture recognition thread
set(GESTURE_SOURCES
    src/gesture_DTW.cpp
)

set(GESTURE_HEADERS
    include/ford_project/gesture_DTW.h
)

# set source, header and the ui for the experiment main control window
set(EXP_WINDOW_SOURCES
    src/exp_main_window.cpp
    src/exp_control.cpp
)

set(EXP_WINDOW_HEADERS
    include/ford_project/exp_main_window.h
)

set(EXP_WINDOW_FORMS
    ui/exp_main_window.ui
)

# set source, header for the aruco vision tracking node
set(VISION_ARUCO_SOURCES
    src/vision_aruco.cpp
)

set(VISION_ARUCO_HEADERS
    include/ford_project/vision_aruco.h
)

# set source, header for post video processing node
set(VIDEO_PROCESSING_SOURCES
    src/video_processing.cpp
)

set(VIDEO_PROCESSING_HEADERS
    include/ford_project/video_processing.h
)

## Declare a C++ executable
add_executable(imu_manager ${IMU_MANAGER_SOURCES} ${IMU_MANAGER_HEADERS})
target_link_libraries(imu_manager ${catkin_LIBRARIES})

add_executable(haptic_test_PCI ${HAPTIC_PCI_SOURCES} ${HAPTIC_PCI_HEADERS})
target_link_libraries(haptic_test_PCI ${catkin_LIBRARIES} 826_64)

add_executable(haptic_USB ${HAPTIC_USB_SOURCES} ${HAPTIC_USB_HEADERS})
target_link_libraries(haptic_USB ${catkin_LIBRARIES} usb-1.0 hidapi-libusb mccusb)

add_executable(gesture_DTW ${GESTURE_SOURCES} ${GESTURE_HEADERS})
target_link_libraries(gesture_DTW ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(vision_aruco ${VISION_ARUCO_SOURCES} ${VISION_ARUCO_HEADERS})
target_link_libraries(vision_aruco ${catkin_LIBRARIES} ${aruco_LIBS} ${OpenCV_LIBRARIES})

add_executable(video_processing ${VIDEO_PROCESSING_SOURCES} ${VIDEO_PROCESSING_HEADERS})
target_link_libraries(video_processing ${catkin_LIBRARIES} ${aruco_LIBS} ${OpenCV_LIBRARIES})

# compile the experiment main window application
qt5_wrap_cpp(MOC_SOURCES ${EXP_WINDOW_HEADERS})
qt5_wrap_ui(FORMS_HEADERS ${EXP_WINDOW_FORMS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

add_executable(exp_control ${EXP_WINDOW_SOURCES} ${EXP_WINDOW_HEADERS} ${MOC_SOURCES} ${FORMS_HEADERS})
target_link_libraries(exp_control ${catkin_LIBRARIES})
qt5_use_modules(exp_control Widgets)

## Add cmake target dependencies of the executable
# add_dependencies(ford_project_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

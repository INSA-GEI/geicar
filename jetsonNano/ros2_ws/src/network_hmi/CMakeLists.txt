cmake_minimum_required(VERSION 3.8)
project(network_hmi)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- Find dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
#find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PkgConfig REQUIRED)

#set(OpenCV_DIR "/opt/ros/$ENV{ROS_DISTRO}/share/opencv4") # Path to ROS OpenCV config
#find_package(OpenCV 4.5 REQUIRED PATHS "/opt/ros/$ENV{ROS_DISTRO}" NO_DEFAULT_PATH)

set(MANUAL_OPENCV_LIBS
    "/lib/aarch64-linux-gnu/libopencv_core.so.4.5d"
    "/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.5d"
    "/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.5d"
    "/lib/aarch64-linux-gnu/libopencv_highgui.so.4.5d"
    # Add others here if you use them (e.g. videoio, calib3d)
)

set(MANUAL_OPENCV_INCLUDE "/usr/include/opencv4")

message(STATUS "OpenCV_FOUND: ${OpenCV_FOUND}")
message(STATUS "OpenCV_VERSION: ${OpenCV_VERSION}")
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")

# This ensures that when the node runs, it looks here first, ignoring /usr/lib
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Add the ROS lib path to the RPATH explicitly
set(CMAKE_INSTALL_RPATH "${OpenCV_LIB_PATH};/opt/ros/$ENV{ROS_DISTRO}/lib")

pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_BASE REQUIRED gstreamer-base-1.0)
pkg_check_modules(GST REQUIRED gstreamer-app-1.0)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTREAMER_BASE_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)

link_directories(
  ${GSTREAMER_LIBRARY_DIRS}
  ${GSTREAMER_BASE_LIBRARY_DIRS}
  ${GST_LIBRARY_DIRS}
)

# ---- Library with components ----
add_library(network_hmi_lib
  src/client_session.cpp
  src/h264_streamer.cpp
  src/shared_client_info.cpp
  src/shared_vehicle_state.cpp
  src/tcp_control_server.cpp
  src/udp_data_receiver.cpp
  src/udp_data_sender.cpp
  src/tcp_udp_bridge_node.cpp
)

target_include_directories(network_hmi_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${MANUAL_OPENCV_INCLUDE}
)

target_link_libraries(network_hmi_lib
  PUBLIC
    nlohmann_json::nlohmann_json
    ${cv_bridge_LIBRARIES}
    #${OpenCV_LIBRARIES}
    ${MANUAL_OPENCV_LIBS}
    ${GSTREAMER_LIBRARIES}
    ${GSTREAMER_BASE_LIBRARIES}
    ${GST_LIBRARIES}
)

ament_target_dependencies(network_hmi_lib
  PUBLIC
  rclcpp
  interfaces
  nav_msgs
  sensor_msgs
  cv_bridge
)

# ---- Executable ----
add_executable(bridge_node src/main.cpp)

target_link_libraries(bridge_node
  network_hmi_lib
)

# ---- Install rules ----
install(TARGETS
  bridge_node
  network_hmi_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()

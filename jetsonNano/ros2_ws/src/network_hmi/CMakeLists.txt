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
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_BASE REQUIRED gstreamer-base-1.0)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTREAMER_BASE_INCLUDE_DIRS}
)

link_directories(
  ${GSTREAMER_LIBRARY_DIRS}
  ${GSTREAMER_BASE_LIBRARY_DIRS}
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
)

target_link_libraries(network_hmi_lib
  PUBLIC
    nlohmann_json::nlohmann_json
    ${OpenCV_LIBRARIES}
    ${GSTREAMER_LIBRARIES}
    ${GSTREAMER_BASE_LIBRARIES}
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

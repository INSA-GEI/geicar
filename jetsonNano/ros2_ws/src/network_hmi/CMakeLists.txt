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

# ---- Library with components ----
add_library(network_hmi_lib
  src/client_session.cpp
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
)

ament_target_dependencies(network_hmi_lib
  PUBLIC
  rclcpp
  interfaces
  nav_msgs
  sensor_msgs
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

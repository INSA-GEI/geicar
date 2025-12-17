cmake_minimum_required(VERSION 3.22)
project(ecosense_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

ament_package()

# Add test executable
add_executable(demo_pick_place
  test/demo_pick_place.cpp
)

ament_target_dependencies(demo_pick_place
  rclcpp
  moveit_ros_planning_interface
)



if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config urdf models DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
install(TARGETS demo_pick_place DESTINATION lib/${PROJECT_NAME})

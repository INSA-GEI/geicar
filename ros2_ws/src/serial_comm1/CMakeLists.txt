cmake_minimum_required(VERSION 3.8)
project(serial_comm1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_executable(talker src/serial_publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs Boost)

add_executable(listener src/serial_subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs Boost)

install(TARGETS 
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.8)
project(drivetrain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Create drivetrain listener executable
add_executable(drivetrain_sub src/drivetrain_sub.cpp)
ament_target_dependencies(drivetrain_sub rclcpp geometry_msgs)

install(TARGETS
  drivetrain_sub
  DESTINATION lib/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.8)
project(tsb_control)

# Enable compiler warnings.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tsb_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# PID controller node.
add_executable(pid_node src/pid_node.cpp)

target_compile_features(pid_node PRIVATE cxx_std_17)
target_include_directories(pid_node PRIVATE include)

ament_target_dependencies(pid_node
  rclcpp
  rcl_interfaces
  geometry_msgs
  nav_msgs
  sensor_msgs
  tsb_interfaces
  tf2
  tf2_geometry_msgs
)

# Sensor processing node.
add_executable(sensor_processing_node src/sensor_processing_node.cpp)

target_compile_features(sensor_processing_node PRIVATE cxx_std_17)
target_include_directories(sensor_processing_node PRIVATE include)

ament_target_dependencies(sensor_processing_node
  rclcpp
  rcl_interfaces
  nav_msgs
  sensor_msgs
  tf2
)

# Path following node.
add_executable(path_following_node src/path_following_node.cpp)

target_compile_features(path_following_node PRIVATE cxx_std_17)
target_include_directories(path_following_node PRIVATE include)

ament_target_dependencies(path_following_node
  rclcpp
  rcl_interfaces
  nav_msgs
  tsb_interfaces
  tf2
  tf2_geometry_msgs
)

# Install executables where ros2 run expects them.
install(TARGETS
  pid_node
  sensor_processing_node
  path_following_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install runtime configuration files.
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Preserve the existing lint configuration.
if(BUILD_TESTING)
  enable_testing()

  # Build and register our standalone algorithm tests.
  foreach(test_name
    pid_test
    heading_estimator_test
    los_guidance_test
    gain_schedule_test
  )
    add_executable(${test_name} test/${test_name}.cpp)

    target_compile_features(${test_name} PRIVATE cxx_std_17)
    target_include_directories(${test_name} PRIVATE include)

    # Keep assert() checks enabled, including in Release builds.
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(${test_name} PRIVATE -UNDEBUG)
    endif()

    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()

  # Preserve the existing lint configuration.
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
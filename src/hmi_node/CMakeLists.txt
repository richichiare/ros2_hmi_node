cmake_minimum_required(VERSION 3.5)
project(hmi_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

#add_executable(hmi_node_server src/main.cpp src/EthernetPrimitives.cpp src/server.cpp)
add_executable(hmi_node_server src/server.cpp)
add_executable(hmi_node_client src/client.cpp)
ament_target_dependencies(hmi_node_server rclcpp geometry_msgs ament_index_cpp yaml_cpp_vendor)
ament_target_dependencies(hmi_node_client rclcpp)

install(TARGETS
  hmi_node_server
  hmi_node_client
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

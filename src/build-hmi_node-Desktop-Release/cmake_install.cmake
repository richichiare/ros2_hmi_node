# Install script for directory: /home/riccardochiaretti/alba_v2_hmi/src/hmi_node

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hmi_node/hmi_node_server" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hmi_node/hmi_node_server")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hmi_node/hmi_node_server"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hmi_node" TYPE EXECUTABLE FILES "/home/riccardochiaretti/alba_v2_hmi/src/build-hmi_node-Desktop-Release/hmi_node_server")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hmi_node/hmi_node_server" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hmi_node/hmi_node_server")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hmi_node/hmi_node_server"
         OLD_RPATH "/home/riccardochiaretti/ros2_eloquent/install/nav_msgs/lib:/home/riccardochiaretti/ros2_eloquent/install/orocos_kdl/lib:/home/riccardochiaretti/ros2_eloquent/install/tf2_ros/lib:/home/riccardochiaretti/ros2_eloquent/install/message_filters/lib:/home/riccardochiaretti/ros2_eloquent/install/rclcpp/lib:/home/riccardochiaretti/ros2_eloquent/install/rcl/lib:/home/riccardochiaretti/ros2_eloquent/install/rcl_interfaces/lib:/home/riccardochiaretti/ros2_eloquent/install/rmw_implementation/lib:/home/riccardochiaretti/ros2_eloquent/install/rmw/lib:/home/riccardochiaretti/ros2_eloquent/install/rcutils/lib:/home/riccardochiaretti/ros2_eloquent/install/rcl_logging_spdlog/lib:/home/riccardochiaretti/ros2_eloquent/install/rcpputils/lib:/home/riccardochiaretti/ros2_eloquent/install/rosgraph_msgs/lib:/home/riccardochiaretti/ros2_eloquent/install/rcl_yaml_param_parser/lib:/home/riccardochiaretti/ros2_eloquent/install/tracetools/lib:/home/riccardochiaretti/ros2_eloquent/install/std_msgs/lib:/home/riccardochiaretti/ros2_eloquent/install/geometry_msgs/lib:/home/riccardochiaretti/ros2_eloquent/install/builtin_interfaces/lib:/home/riccardochiaretti/ros2_eloquent/install/unique_identifier_msgs/lib:/home/riccardochiaretti/ros2_eloquent/install/rosidl_typesupport_cpp/lib:/home/riccardochiaretti/ros2_eloquent/install/rosidl_typesupport_c/lib:/home/riccardochiaretti/ros2_eloquent/install/rosidl_typesupport_introspection_cpp/lib:/home/riccardochiaretti/ros2_eloquent/install/rosidl_typesupport_introspection_c/lib:/home/riccardochiaretti/ros2_eloquent/install/rosidl_generator_c/lib:/home/riccardochiaretti/ros2_eloquent/install/action_msgs/lib:/home/riccardochiaretti/ros2_eloquent/install/tf2_msgs/lib:/home/riccardochiaretti/ros2_eloquent/install/tf2/lib:/home/riccardochiaretti/ros2_eloquent/install/ament_index_cpp/lib:/home/riccardochiaretti/ros2_eloquent/install/yaml_cpp_vendor/opt/yaml_cpp_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hmi_node/hmi_node_server")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hmi_node/" TYPE DIRECTORY FILES "/home/riccardochiaretti/alba_v2_hmi/src/hmi_node/launch")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/riccardochiaretti/alba_v2_hmi/src/build-hmi_node-Desktop-Release/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/hmi_node")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/riccardochiaretti/alba_v2_hmi/src/build-hmi_node-Desktop-Release/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/hmi_node")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hmi_node/environment" TYPE FILE FILES "/home/riccardochiaretti/ros2_eloquent/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hmi_node/environment" TYPE FILE FILES "/home/riccardochiaretti/alba_v2_hmi/src/build-hmi_node-Desktop-Release/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hmi_node/environment" TYPE FILE FILES "/home/riccardochiaretti/ros2_eloquent/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hmi_node/environment" TYPE FILE FILES "/home/riccardochiaretti/alba_v2_hmi/src/build-hmi_node-Desktop-Release/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hmi_node" TYPE FILE FILES "/home/riccardochiaretti/alba_v2_hmi/src/build-hmi_node-Desktop-Release/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hmi_node" TYPE FILE FILES "/home/riccardochiaretti/alba_v2_hmi/src/build-hmi_node-Desktop-Release/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hmi_node" TYPE FILE FILES "/home/riccardochiaretti/alba_v2_hmi/src/build-hmi_node-Desktop-Release/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hmi_node" TYPE FILE FILES "/home/riccardochiaretti/alba_v2_hmi/src/build-hmi_node-Desktop-Release/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hmi_node" TYPE FILE FILES "/home/riccardochiaretti/alba_v2_hmi/src/build-hmi_node-Desktop-Release/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/riccardochiaretti/alba_v2_hmi/src/build-hmi_node-Desktop-Release/ament_cmake_index/share/ament_index/resource_index/packages/hmi_node")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hmi_node/cmake" TYPE FILE FILES
    "/home/riccardochiaretti/alba_v2_hmi/src/build-hmi_node-Desktop-Release/ament_cmake_core/hmi_nodeConfig.cmake"
    "/home/riccardochiaretti/alba_v2_hmi/src/build-hmi_node-Desktop-Release/ament_cmake_core/hmi_nodeConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hmi_node" TYPE FILE FILES "/home/riccardochiaretti/alba_v2_hmi/src/hmi_node/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/riccardochiaretti/alba_v2_hmi/src/build-hmi_node-Desktop-Release/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")

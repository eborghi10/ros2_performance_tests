cmake_minimum_required(VERSION 3.5)

project(throughput_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(realsense2_camera REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_library(talker_component SHARED
  src/talker_component.cpp)
target_compile_definitions(talker_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(talker_component
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs")
rclcpp_components_register_nodes(talker_component "throughput_test::Talker")
set(node_plugins "${node_plugins}throughput_test::Talker;$<TARGET_FILE:talker_component>\n")


add_library(listener_component SHARED
  src/listener_component.cpp)
target_compile_definitions(listener_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(listener_component
  "realsense2_camera"
  "realsense2_camera_msgs"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs")
rclcpp_components_register_nodes(listener_component "throughput_test::Listener")
set(node_plugins "${node_plugins}throughput_test::Listener;$<TARGET_FILE:listener_component>\n")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
ament_environment_hooks(
  "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")

install(TARGETS
  listener_component
  talker_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install launch files.
install(DIRECTORY
  config
  data
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

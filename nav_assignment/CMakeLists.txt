cmake_minimum_required(VERSION 3.8)
project(nav_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Find our custom interfaces package!
find_package(nav_interfaces REQUIRED)

# -----------------------------------------------------------------
# 1. Build the Action Client Component
# -----------------------------------------------------------------
add_library(nav_client_component SHARED src/nav_client_component.cpp)
target_compile_features(nav_client_component PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(nav_client_component
  rclcpp
  rclcpp_action
  rclcpp_components
  nav_interfaces
)
rclcpp_components_register_node(nav_client_component
  PLUGIN "nav_assignment::NavClient"
  EXECUTABLE nav_client_exe
)

# -----------------------------------------------------------------
# 2. Build the Action Server Component
# -----------------------------------------------------------------
add_library(nav_server_component SHARED src/nav_server_component.cpp)
target_compile_features(nav_server_component PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(nav_server_component
  rclcpp
  rclcpp_action
  rclcpp_components
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  std_msgs
  nav_interfaces
)
rclcpp_components_register_node(nav_server_component
  PLUGIN "nav_assignment::NavServer"
  EXECUTABLE nav_server_exe
)

# Install the launch directory (we will create this later)

# Install the compiled shared libraries
install(TARGETS
  nav_client_component
  nav_server_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

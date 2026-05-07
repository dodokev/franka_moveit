cmake_minimum_required(VERSION 3.8)
project(franka_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)

add_library(custom_feasibility_adapter SHARED
  src/custom_feasibility_adapter.cpp
)

ament_target_dependencies(custom_feasibility_adapter
  rclcpp
  pluginlib
  moveit_core
  moveit_ros_planning
)

pluginlib_export_plugin_description_file(
  moveit_core
  custom_feasibility_adapter.xml
)

install(TARGETS custom_feasibility_adapter
  DESTINATION lib
)

install(FILES custom_feasibility_adapter.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(pluginlib moveit_core moveit_ros_planning)

# ================================================================================

add_executable(franka_moveit src/franka_moveit.cpp)
target_include_directories(franka_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(franka_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  franka_moveit
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
)

install(TARGETS franka_moveit
  DESTINATION lib/${PROJECT_NAME})
  
# ================================================================================
  
  add_executable(testing src/testing.cpp)
  target_include_directories(testing PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(testing PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  
  ament_target_dependencies(
    testing
    "rclcpp"
    "moveit_ros_planning_interface"
    "moveit_visual_tools"
  )
  
  install(TARGETS testing
    DESTINATION lib/${PROJECT_NAME})
  
# ================================================================================

  add_executable(cloud_transform src/cloud_transform.cpp)
  target_include_directories(cloud_transform PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(cloud_transform PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  
  ament_target_dependencies(
    cloud_transform
    "rclcpp"
    "moveit_ros_planning_interface"
    "moveit_visual_tools"
  )
  
  install(TARGETS cloud_transform
    DESTINATION lib/${PROJECT_NAME})
  
# ======================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch config include
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

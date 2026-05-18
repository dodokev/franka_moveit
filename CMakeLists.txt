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
find_package(franka_moveit_msg REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

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
  
  add_executable(explore_nullspace src/explore_nullspace.cpp)
  target_include_directories(explore_nullspace PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(explore_nullspace PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  
  ament_target_dependencies(
    explore_nullspace
    "rclcpp"
    "moveit_ros_planning_interface"
    "moveit_visual_tools"
  )
  
  install(TARGETS explore_nullspace
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
  
# ============================================================================

add_executable(simple_mvt src/simple_mvt.cpp)
target_include_directories(simple_mvt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(simple_mvt PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  simple_mvt
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  franka_moveit_msg
)

install(TARGETS simple_mvt
  DESTINATION lib/${PROJECT_NAME})
  
# ===============================================================================

add_executable(bridge_franka src/bridge_franka.cpp)
target_include_directories(bridge_franka PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(bridge_franka PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  bridge_franka
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  franka_moveit_msg
)

install(TARGETS bridge_franka
  DESTINATION lib/${PROJECT_NAME})

# ================================================================================
# ===============================================================================

add_executable(outlier_filter src/outlier_filter.cpp)
target_include_directories(outlier_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(outlier_filter PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  outlier_filter
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  pcl_ros
)

install(TARGETS outlier_filter
  DESTINATION lib/${PROJECT_NAME})

# ================================================================================

add_executable(octomap_cleaner src/octomap_cleaner.cpp)
target_include_directories(octomap_cleaner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(octomap_cleaner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  octomap_cleaner
  "rclcpp"
  "moveit_ros_planning_interface"
  pcl_ros
)

install(TARGETS octomap_cleaner
  DESTINATION lib/${PROJECT_NAME})

# ================================================================================

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

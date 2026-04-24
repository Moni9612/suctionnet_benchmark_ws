cmake_minimum_required(VERSION 3.8)
project(move_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)  # Ensure tf2_ros is found
find_package(tf2_eigen REQUIRED)  # Add tf2_eigen for transformations

add_executable(move_robot src/move_robot.cpp)

# Include directories
target_include_directories(move_robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
  ${moveit_ros_planning_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}  # Include tf2_ros headers
  ${tf2_eigen_INCLUDE_DIRS}  # Include tf2_eigen headers
)

# Compile features
target_compile_features(move_robot PUBLIC c_std_99 cxx_std_17)

# Link dependencies
ament_target_dependencies(
  move_robot
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_ros_planning"
  "tf2_ros"  # Ensure tf2_ros is linked
  "tf2_eigen"  # Link tf2_eigen for transformations
)

# Compile features
target_compile_features(move_robot PUBLIC c_std_99 cxx_std_17)

target_link_libraries(move_robot
  ${moveit_ros_planning_interface_LIBRARIES}
  ${moveit_ros_planning_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${tf2_ros_LIBRARIES}  # Ensure tf2_ros is linked
  ${tf2_eigen_LIBRARIES}  # Ensure tf2_eigen is linked
  ${tf2_LIBRARIES} 
)

install(TARGETS move_robot
  DESTINATION lib/${PROJECT_NAME})
  
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


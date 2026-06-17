cmake_minimum_required(VERSION 3.5)
project(gazebo_mecanum_plugins)

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
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(osqp_vendor REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#add_executable(omniblue_mpc_controller
#  src/omniblue_mpc_controller.cpp
#)

add_executable(omniblue_smc_controller
  src/omniblue_smc_controller.cpp
)

add_executable(
  formation_smc_controller
  src/formation_smc_controller.cpp
)

add_executable(
    formation_manager
    src/formation_manager.cpp
)

add_library(gazebo_mecanum_drive SHARED
  src/gazebo_mecanum_drive.cpp)

add_library(gazebo_mecanum_pid_drive SHARED
  src/gazebo_mecanum_pid_drive.cpp)

target_include_directories(gazebo_mecanum_drive PUBLIC include)
target_include_directories(gazebo_mecanum_pid_drive PUBLIC include)
target_include_directories(omniblue_smc_controller PUBLIC include)

ament_target_dependencies(gazebo_mecanum_drive
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)

ament_target_dependencies(gazebo_mecanum_pid_drive
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)

#ament_target_dependencies(omniblue_mpc_controller
#  rclcpp
#  geometry_msgs
#  nav_msgs
#  sensor_msgs
#  tf2
#  tf2_geometry_msgs
#  osqp_vendor
#)

ament_target_dependencies(omniblue_smc_controller
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(
  formation_smc_controller
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(
    formation_manager
    rclcpp
    geometry_msgs
)

target_link_libraries(
  formation_smc_controller
  Eigen3::Eigen
)

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
        gazebo_mecanum_drive
        gazebo_mecanum_pid_drive
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  #omniblue_mpc_controller
  omniblue_smc_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
  formation_smc_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
    TARGETS
    formation_smc_controller
    formation_manager
    DESTINATION lib/${PROJECT_NAME}
)


ament_package()

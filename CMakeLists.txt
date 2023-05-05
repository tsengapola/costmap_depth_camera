cmake_minimum_required(VERSION 3.5)
project(costmap_depth_camera)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to Release build
if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()
message( STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}" )

set(lib_name ${PROJECT_NAME})

# === Dependencies ===

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

find_package(PCL REQUIRED COMPONENTS filters common)
if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  # This package fails to build on Debian Stretch with a linking error against
  # 'Qt5::Widgets'.  This is a transitive dependency that comes in to PCL via
  # the PCL dependency on VTK.  However, we don't actually care about the Qt
  # dependencies for this package, so just remove them.  This is similar to the
  # workaround in https://github.com/ros-perception/perception_pcl/pull/151,
  # and can be removed when Stretch goes out of support.
  list(REMOVE_ITEM PCL_LIBRARIES
    "vtkGUISupportQt"
    "vtkGUISupportQtOpenGL"
    "vtkGUISupportQtSQL"
    "vtkGUISupportQtWebkit"
    "vtkViewsQt"
    "vtkRenderingQt")
endif()

set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    pluginlib
)

# === Build ===
include_directories(
  include
  ${BOOST_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(${lib_name} SHARED
            src/observation.cpp
            src/observation_buffer.cpp
            plugins/depth_camera_obstacle_layer.cpp)

target_link_libraries(${lib_name} "${cpp_typesupport_target}")
target_compile_definitions(${lib_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

target_link_libraries(${lib_name}
  ${Boost_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(${lib_name}
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  nav2_costmap_2d
  message_filters
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  tf2_ros
  pluginlib
  builtin_interfaces
  tf2_geometry_msgs
  tf2_sensor_msgs
  builtin_interfaces
)

# === Installation ===
install(TARGETS ${lib_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  ament_lint_auto_find_test_dependencies()
#endif()

# === Ament work ===

# Install the plugin
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()

#ament_auto_package(INSTALL_TO_SHARE launch)



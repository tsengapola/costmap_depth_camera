cmake_minimum_required(VERSION 3.0.2)
project(costmap_depth_camera)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            dynamic_reconfigure
            geometry_msgs
            map_msgs
            message_filters
            message_generation
            nav_msgs
            pluginlib
            roscpp
            sensor_msgs
            std_msgs
            tf2
            tf2_geometry_msgs
            tf2_ros
            tf2_sensor_msgs
            visualization_msgs
            costmap_2d
            pcl_ros
        )

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

# messages
#add_message_files(
#    DIRECTORY msg
#    FILES
#)

generate_messages(
    DEPENDENCIES
        std_msgs
        geometry_msgs
        map_msgs
)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/DepthCameraPlugin.cfg
)

catkin_package(
    INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIRS}
    LIBRARIES layers
    CATKIN_DEPENDS
        dynamic_reconfigure
        geometry_msgs
        map_msgs
        message_filters
        message_runtime
        nav_msgs
        pluginlib
        roscpp
        sensor_msgs
        std_msgs
        tf2_ros
        visualization_msgs
        costmap_2d
        pcl_ros
    DEPENDS
        EIGEN3
        Boost
        PCL
)

add_library(costmap_depth_camera
  plugins/depth_camera_obstacle_layer.cpp
  src/observation_buffer.cpp
)
add_dependencies(costmap_depth_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(costmap_depth_camera
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)


install(TARGETS
    costmap_depth_camera
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

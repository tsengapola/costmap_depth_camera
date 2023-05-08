import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    
    # Get the launch directory
    robot_name = ''
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    aws_small_warehouse_dir = get_package_share_directory('aws_robomaker_small_warehouse_world') # aws related files such as maps, worlds, urdf
    costmap_depth_camera_dir = get_package_share_directory('costmap_depth_camera') # for nav2_param, rviz files

    print('+'*80)
    print(aws_small_warehouse_dir)
    print(costmap_depth_camera_dir)
    print('+'*80)

    # Create the launch configuration variables
    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    namespace_class = TextSubstitution(text=robot_name)
    namespace_string = robot_name
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value=TextSubstitution(text=robot_name),
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='True',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(aws_small_warehouse_dir, 'maps', 'ware_house.yaml'),
        description='Full path to map file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(costmap_depth_camera_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(costmap_depth_camera_dir, 'rviz', 'ros2_tb3_sim.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(aws_small_warehouse_dir, 'worlds', 'no_roof_small_warehouse', 'aws_nvblox.world'),
        description='Full path to world model file to load')

    aws_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(aws_small_warehouse_dir, 'launch', 'aws_nvblox_warehouse_launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'use_sim_time': use_sim_time,
                          'use_simulator': use_simulator,
                          'headless': headless}.items())


    tf_map2odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen" ,
        arguments=["0.0", "0.0", "0.0", "0.0", "0", "0", "map", "odom"]
    )

    # voxcloud
    voxcloud_node = Node(
        package='voxcloud_ros',
        executable='striker_node',
        output='screen',
        remappings=[
            ("/camera_left/depth/image_rect_raw", "/depth_camera_left/camera/depth/image_raw"),
            ("/camera_left/depth/camera_info", "/depth_camera_left/camera/depth/camera_info"),
            ("/camera_right/depth/image_rect_raw", "/depth_camera_right/camera/depth/image_raw"),
            ("/camera_right/depth/camera_info", "/depth_camera_right/camera/depth/camera_info")
        ]
    )

    #spawn robot by .sdf file
    sdf_model = LaunchConfiguration('sdf_model')
    sdf_model_path = os.path.join(aws_small_warehouse_dir, 'models', 'waffle_realsense', 'turtlebot3_waffle', 'model.sdf')

    declare_sdf_model_path_cmd = DeclareLaunchArgument(
        name='sdf_model', 
        default_value=sdf_model_path, 
        description='Absolute path to robot sdf file')

    spawn_robot_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name, 
               '-reference_frame', 'map',
               '-file', sdf_model,
               '-robot_namespace', robot_name, 
                  '-x', '-1.0',
                  '-y', '0.0',
                  '-z', '0.0',
                  '-Y', '0.0'],
                  output='screen')
    
    urdf = os.path.join(aws_small_warehouse_dir, 'models', 'waffle_realsense', 'urdf', 'turtlebot3_waffle.urdf')
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace_class,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings,
        arguments=[urdf])

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': namespace_string,
                          'use_namespace': 'True',
                          'rviz_config': rviz_config_file}.items())

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace_string,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'autostart': autostart}.items())


    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(aws_cmd)
    ld.add_action(tf_map2odom)
    #ld.add_action(voxcloud_node)
    #ld.add_action(nvblox_config)
    #ld.add_action(nvblox_node)

    ld.add_action(declare_sdf_model_path_cmd)

    ld.add_action(spawn_robot_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)

    return ld
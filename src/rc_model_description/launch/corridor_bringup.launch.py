# rc_model_description/launch/corridor_bringup.launch.py
# Launch RC car in TurtleBot3 narrow corridor world
import os, tempfile
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, OpaqueFunction, ExecuteProcess, TimerAction)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def _prepare_and_spawn(context, *args, **kwargs):
    pkg_share = get_package_share_directory('rc_model_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'rc_model_ros2.xacro')

    # 1) xacro -> URDF string (fresh every launch)
    urdf_xml = xacro.process_file(xacro_file).toxml()

    # 2) Start robot_state_publisher (publishes /robot_description)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(urdf_xml, value_type=str),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    # 3) Spawn from /robot_description with a height
    model_name = LaunchConfiguration('model_name').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)
    spawn_z    = LaunchConfiguration('spawn_z').perform(context)
    spawn_x    = LaunchConfiguration('spawn_x').perform(context)
    spawn_y    = LaunchConfiguration('spawn_y').perform(context)
    spawn_yaw    = LaunchConfiguration('spawn_yaw').perform(context)

    spawn = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', world_name,
            '-name', model_name,
            '-topic', '/robot_description',
            '-z', spawn_z,
            '-x', spawn_x,
            '-y', spawn_y,
            '-Y', spawn_yaw,
        ],
        output='screen'
    )

    # 4) Optional RViz
    rviz_cfg = os.path.join(pkg_share, 'rviz', 'urdf.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg] if os.path.exists(rviz_cfg) else [],
        parameters=[{'use_sim_time': True}], 
        output='screen'
    )
    
    controllers_yaml_path = LaunchConfiguration('controllers_yaml').perform(context)
    controller_mgr_ns = '/controller_manager'

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", controller_mgr_ns],
        output="screen",
    )

    ackermann_steering_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_controller", "--controller-manager", controller_mgr_ns,"--param-file", controllers_yaml_path],
        output="screen",
    )

    spawn_robot = TimerAction(period=1.0, actions=[spawn])
    spawn_jsb   = TimerAction(period=3.0, actions=[joint_state_broadcaster_spawner])
    spawn_ack   = TimerAction(period=4.5, actions=[ackermann_steering_spawner])

    return [rsp, spawn_robot, spawn_jsb, spawn_ack, rviz]

def generate_launch_description():
    pkg_share   = FindPackageShare('rc_model_description')
    ros_gz_share= FindPackageShare('ros_gz_sim')
    
    # Use our custom Ignition-compatible narrow corridor world
    world_path = PathJoinSubstitution([pkg_share, 'worlds', 'narrow_corridor.sdf'])

    world_name   = DeclareLaunchArgument('world_name', default_value='narrow_corridor')
    model_name   = DeclareLaunchArgument('model_name', default_value='rc')
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    spawn_z      = DeclareLaunchArgument('spawn_z', default_value='0.12',
                         description='Spawn height (m) to avoid ground clipping')
    # Spawn in west corridor like TurtleBot3: x=-3.5, y=0.0, facing east (yaw=0)
    spawn_y      = DeclareLaunchArgument('spawn_y', default_value='0.0',
                         description='Spawn Y (m) - centered in corridor')
    spawn_x      = DeclareLaunchArgument('spawn_x', default_value='-3.5',
                         description='Spawn X (m) - west corridor (between outer -4 and inner -2)')
    spawn_yaw      = DeclareLaunchArgument('spawn_yaw', default_value='0.0',
                         description='Spawn yaw (rad) - facing east')
    
    # Resource paths (models & worlds)
    set_gz_res  = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=PathJoinSubstitution([pkg_share, 'models'])
    )
    set_ign_res = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=PathJoinSubstitution([pkg_share, 'models'])
    )

    controllers_yaml = DeclareLaunchArgument(
    'controllers_yaml',
    default_value=PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml']),
    description='Controllers YAML for ros2_control controllers'
    )

    # Start Gazebo with narrow corridor world
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_share, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': world_path}.items()
    )

    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
        output='screen'
    )

    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        output='screen',
        arguments=['/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'],
    )
    
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
    )

    gt_pose_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name='gt_pose_bridge',
    output='screen',
    arguments=[
        '/world/default/model/rc/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
    ],
    remappings=[
        ('/world/default/model/rc/pose', '/ground_truth_pose'),
    ],
    parameters=[{'use_sim_time': True}],
    )

    # Camera bridge - publishes RGB images to /camera/image_raw
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        output='screen',
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        remappings=[
            ('/camera', '/camera/image_raw'),
        ],
        parameters=[{'use_sim_time': True}],
    )

    # Stamper bridge: Converts /cmd_vel (Twist) → /ackermann_steering_controller/reference (TwistStamped)
    # This allows teleop_twist_keyboard to control the RC car
    stamper_bridge = Node(
        package='rc_nav_bridge',
        executable='stamper',
        name='twist_to_stamped',
        output='screen',
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('reference', '/ackermann_steering_controller/reference'),
        ],
        parameters=[{'frame_id': ''}],
    )

    return LaunchDescription([
        world_name, model_name, use_sim_time, spawn_z, spawn_x, spawn_y, spawn_yaw,
        set_gz_res, set_ign_res, controllers_yaml,
        gz_launch,
        OpaqueFunction(function=_prepare_and_spawn),
        scan_bridge,
        imu_bridge,
        clock_bridge,
        gt_pose_bridge,
        camera_bridge,
        stamper_bridge,
    ])

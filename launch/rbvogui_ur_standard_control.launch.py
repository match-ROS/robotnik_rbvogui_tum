from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    robot_id = LaunchConfiguration('robot_id')
    use_sim_time = LaunchConfiguration('use_sim_time')
    controller_config = PathJoinSubstitution([
        FindPackageShare('robotnik_rbvogui_tum'),
        'config',
        'rbvogui_standard_controllers.yaml',
    ])
    world_path = PathJoinSubstitution([
        FindPackageShare('robotnik_gazebo_ignition'),
        'worlds',
        [LaunchConfiguration('world'), '.world'],
    ])
    gui_enabled = PythonExpression([
        "'", LaunchConfiguration('gui'),
        "'.strip().lower() in ('true','1','yes','on')",
    ])
    # A GUI launched from the Snap build of VS Code inherits GTK/Qt paths
    # pointing into /snap. Gazebo then mixes its host libc with Snap's
    # libpthread and aborts before the simulation can be measured. These
    # variables affect GUI discovery only, so remove them solely for gui:=true.
    gui_environment_cleanup = [
        UnsetEnvironmentVariable(name, condition=IfCondition(gui_enabled))
        for name in (
            'GTK_EXE_PREFIX', 'GTK_IM_MODULE_FILE', 'GTK_MODULES', 'GTK_PATH',
            'QT_ACCESSIBILITY', 'QT_IM_MODULE', 'XDG_DATA_HOME', 'XDG_DATA_DIRS',
        )
    ]

    world_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py',
        ])),
        launch_arguments={
            'gz_args': ['-r ', world_path],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(gui_enabled),
    )

    world_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py',
        ])),
        launch_arguments={
            'gz_args': ['-r -s ', world_path],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=UnlessCondition(gui_enabled),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('robotnik_description'),
            'launch',
            'robot_description.launch.py',
        ])),
        launch_arguments={
            'verbose': 'false',
            'robot_xacro_path': PathJoinSubstitution([
                FindPackageShare('robotnik_rbvogui_tum'),
                'urdf',
                'rbvogui_ur_standard_control.urdf.xacro',
            ]),
            'frame_prefix': [robot_id, '_'],
            'namespace': robot_id,
            'gazebo_ignition': 'true',
            'arm_type': LaunchConfiguration('arm_type'),
            'low_performance_simulation': 'true',
        }.items(),
    )

    create_robot = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=robot_id,
        arguments=[
            '-name', robot_id,
            '-topic', 'robot_description',
            '-robot_namespace', robot_id,
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
        ],
        output='screen',
    )

    model_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            [
                '/world/robotnik_simple/dynamic_pose/info'
                '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
        ],
        output='screen',
    )

    robot_pose_publisher = Node(
        package='robotnik_rbvogui_tum',
        executable='tf_model_pose_to_pose_stamped.py',
        name='rbvogui_robot_pose_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_robot_pose')),
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/world/robotnik_simple/dynamic_pose/info',
            'output_topic': '/robot_pose',
            'model_frame': robot_id,
            'world_frame': 'robotnik_simple',
            'output_frame': 'map',
            'fallback_transform_index': 0,
            'publish_tf': True,
            'tf_child_frame': [robot_id, '_base_footprint'],
        }],
    )

    tcp_pose_publisher = Node(
        package='robotnik_rbvogui_tum',
        executable='current_pose_from_tf.py',
        name='rbvogui_current_tcp_pose_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': 'map',
            'source_frame': [robot_id, '_arm_nozzle_tip'],
            'pose_topic': '/current_tcp_pose',
            'publish_rate': 20.0,
        }],
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_id,
        arguments=[
            '--controller-manager-timeout', '60',
            '--service-call-timeout', '60',
            '--param-file', controller_config,
            'joint_state_broadcaster',
            'steering_position_controller',
            'wheel_velocity_controller',
            'lift_position_controller',
            'joint_trajectory_controller',
        ],
        output='screen',
    )

    arm_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_id,
        arguments=[
            '--controller-manager-timeout', '60',
            '--service-call-timeout', '60',
            '--param-file', controller_config,
            '--inactive',
            'arm_forward_velocity_controller',
        ],
        output='screen',
    )

    lift_initial_command = ExecuteProcess(
        cmd=[
            'ros2',
            'topic',
            'pub',
            '--once',
            ['/', robot_id, '/lift_position_controller/commands'],
            'std_msgs/msg/Float64MultiArray',
            ['{data: [', LaunchConfiguration('lift_initial_position'), ']}'],
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('set_lift_initial_position')),
    )

    swerve_controller = Node(
        package='robotnik_rbvogui_tum',
        executable='rbvogui_swerve_controller.py',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'cmd_vel_topic': [
                '/', robot_id, '/robotnik_base_control/cmd_vel_unstamped'
            ],
            'steering_command_topic': [
                '/', robot_id, '/steering_position_controller/commands'
            ],
            'wheel_command_topic': [
                '/', robot_id, '/wheel_velocity_controller/commands'
            ],
            'joint_states_topic': ['/', robot_id, '/joint_states'],
            'joint_prefix': [robot_id, '_'],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='robot'),
        DeclareLaunchArgument('arm_type', default_value='ur20'),
        DeclareLaunchArgument('world', default_value='empty'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('publish_robot_pose', default_value='true'),
        DeclareLaunchArgument('set_lift_initial_position', default_value='true'),
        DeclareLaunchArgument('lift_initial_position', default_value='0.0'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.1'),
        *gui_environment_cleanup,
        world_with_gui,
        world_headless,
        clock_bridge,
        robot_description,
        TimerAction(period=2.0, actions=[create_robot]),
        TimerAction(period=3.0, actions=[model_pose_bridge]),
        TimerAction(period=3.5, actions=[robot_pose_publisher]),
        TimerAction(period=4.0, actions=[controller_spawner]),
        TimerAction(period=4.5, actions=[arm_velocity_controller_spawner]),
        TimerAction(period=5.5, actions=[lift_initial_command]),
        TimerAction(period=7.0, actions=[tcp_pose_publisher]),
        TimerAction(period=8.0, actions=[swerve_controller]),
    ])

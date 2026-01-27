from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, 
                            RegisterEventHandler, SetEnvironmentVariable, TimerAction)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # --- 1. CONFIGURAZIONE PERCORSI MODELLI (ARUCO) ---
    pkg_robot_sim = get_package_share_directory('robot_sim')
    models_path = os.path.join(pkg_robot_sim, 'models')

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_path
    )
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=models_path 
    )

    # --- 2. ENVIRONMENT: ros2_control plugin ---
    set_plugin_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value=[
            '/opt/ros/humble/lib:',
            os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')
        ]
    )

    # --- 3. IIWA ARGUMENTS & DESCRIPTION ---
    iiwa_description_arg = DeclareLaunchArgument(
        'iiwa_description_file',
        default_value='iiwa.config.xacro'
    )

    config_file = LaunchConfiguration('iiwa_description_file')

    xacro_path = PathJoinSubstitution([
        FindPackageShare('iiwa_description'),
        'config',
        config_file
    ])

    robot_description = {
        'robot_description': ParameterValue(Command([
            'xacro ', xacro_path,
            ' prefix:=iiwa_',
            ' parent:=world',
            ' use_sim:=true',
            ' namespace:=iiwa',
            ' command_interface:=velocity'
        ]), value_type=str)
    }

    # --- 4. FRA2MO ROBOT DESCRIPTION ---
    fra2mo_xacro_path = PathJoinSubstitution([
        FindPackageShare('ros2_fra2mo'),
        'urdf',
        'fra2mo.urdf.xacro'
    ])

    robot_description2 = {
        'robot_description': ParameterValue(Command([
            'xacro ', fra2mo_xacro_path,
        ]), value_type=str)
    }

    # --- 5. GAZEBO WORLD ---
    world_path = os.path.join(pkg_robot_sim, 'worlds', 'my_custom_world.sdf')

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_path, '-v', '4', '-r'],
        output='screen'
    )

    # --- 6. NODES: State Publishers ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='iiwa',
        parameters=[robot_description, {'use_sim_time': True}],
        remappings=[('/robot_description', 'robot_description')],
        output='screen'
    )
    
    robot_state_publisher2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description2, {'use_sim_time': True}],
        namespace='fra2mo',
        output='screen'
    )

    # --- 7. BRIDGE ---
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/model/fra2mo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/model/fra2mo/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/model/aruco_tag/detachable_joint/attach@std_msgs/msg/Empty]ignition.msgs.Empty',
            '/model/aruco_tag/detachable_joint/detach@std_msgs/msg/Empty]ignition.msgs.Empty',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    odom_tf = Node(
        package='ros2_fra2mo',
        executable='dynamic_tf_publisher',
        name='odom_tf',
        parameters=[{"use_sim_time": True}],
        output='screen'
    )

    # --- 8. SPAWN ROBOTS ---
    spawn_iiwa = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/iiwa/robot_description',
            '-name', 'iiwa',
            '-x', '0', '-y', '0', '-z', '0'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    spawn_fra2mo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/fra2mo/robot_description',
            '-name', 'fra2mo',
            '-x', '1.5', '-y', '0', '-z', '0'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # --- 9. CONTROLLERS ---
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        namespace='iiwa',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['velocity_controller', '-c', '/iiwa/controller_manager'],
        namespace='iiwa',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_iiwa,
            on_exit=[joint_state_broadcaster, velocity_controller]
        )
    )

    # --- 10. DETACH LOGIC ---

    # Definiamo il comando di pubblicazione
    detach_package = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/model/aruco_tag/detachable_joint/detach', 'std_msgs/msg/Empty', '{}', '-w', '0'],
        output='screen'
    )

    # Invece di OnProcessStart del bridge, aspettiamo che lo spawner dei controller finisca.
    # Questo garantisce che il robot sia presente, i controller siano attivi e il mondo sia "stabile".
    detach_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=velocity_controller, # Quando il controller è pronto, il mondo lo è sicuramente
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[detach_package]
                )
            ]
        )
    )

    # --- 11. RETURN DESCRIPTION ---
    return LaunchDescription([
        set_gz_resource_path,
        set_ign_resource_path,
        set_plugin_path,
        iiwa_description_arg,
        gazebo,
        robot_state_publisher,
        robot_state_publisher2,
        bridge,
        odom_tf,
        spawn_iiwa,
        spawn_fra2mo,
        controllers_after_spawn,
        detach_handler
    ])

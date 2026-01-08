import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Percorso del file dei parametri
    config_params = PathJoinSubstitution([
        FindPackageShare('ros2_kdl_package'),
        'config',
        'kdl_params.yaml'
    ])

    # --- ARGOMENTI ---
    cmd_interface_arg = DeclareLaunchArgument(
        'cmd_interface', 
        default_value='velocity',
        description='Select controller: position, velocity or effort'
    )
    
    ctrl_arg = DeclareLaunchArgument(
        'ctrl', 
        default_value='velocity_ctrl_null',
        description='Select velocity controller: velocity_ctrl, velocity_ctrl_null or vision'
    )

    # --- NODO BRIDGE (IGNITION -> ROS 2) ---
    # Questo nodo traduce i topic della camera per ROS
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'
        ],
        output='screen'
    )

    # --- NODO ARUCO DETECTION ---
    # Questo nodo analizza le immagini e pubblica /aruco_single/pose
    aruco_node = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        parameters=[{
            'image_is_rect': True,
            'marker_size': 0.1,         # Dimensione del tag in metri
            'marker_id': 201,            # ID del tag (assicurati coincida con Gazebo)
            'reference_frame': 'camera_link_optical',
            'camera_frame': 'camera_link_optical',
            'marker_frame': 'aruco_marker_frame'
        }],
        remappings=[
            ('/camera_info', '/camera/camera_info'),
            ('/image', '/camera/image_raw'),
            ('/aruco_single/pose', '/aruco_single/pose')
        ],
        output='screen'
    )

    # --- NODO KDL (IL TUO CODICE C++) ---
    ros2_kdl_node = Node(
        package='ros2_kdl_package',
        executable='ros2_kdl_node',
        namespace='iiwa',
        output='screen',
        parameters=[
            config_params,
            {
                'use_sim_time': True,
                'cmd_interface': LaunchConfiguration('cmd_interface'),
                'ctrl': LaunchConfiguration('ctrl'),
            }
        ],
        remappings=[
            ('joint_states', '/iiwa/joint_states'),
            ('velocity_controller/commands', '/iiwa/velocity_controller/commands'),
            # Mapping per il topic della posa del marker
            ('/aruco_single/pose', '/aruco_single/pose')
        ]
    )

    return LaunchDescription([
        cmd_interface_arg,
        ctrl_arg,
        bridge_node,
        aruco_node,
        ros2_kdl_node
    ])

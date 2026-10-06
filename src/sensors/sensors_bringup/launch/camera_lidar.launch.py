"""Launch the Spinnaker RGB camera and the Ouster lidar together."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('spinnaker_camera_driver'), 'launch', 'driver_node.launch.py']
            )
        ),
        launch_arguments={'serial': LaunchConfig('serial')}.items(),
    )
    image_rect_node = Node(
        package="image_proc",
        executable="rectify_node",
        name="image_rect",
        remappings=[
            ("image", "/flir_camera/image_raw"),
            ("camera_info", "/flir_camera/camera_info"),
            ("image_rect", "/camera/rgb/image_rect",)
        ]
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ouster_ros'), 'launch', 'driver.launch.py'])
        ),
        launch_arguments={
            'viz': LaunchConfig('viz'),
            'ouster_ns': 'lidar',
            # node key in this file must match ouster_ns (lidar/os_driver)
            'params_file': PathJoinSubstitution(
                [FindPackageShare('sensors_bringup'), 'config', 'ouster_params.yaml']
            ),
            'rviz_config': PathJoinSubstitution(
                [FindPackageShare('sensors_bringup'), 'config', 'lidar.rviz']
            ),
        }.items(),
    )

    lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('rko_lio'), 'launch', 'odometry.launch.py'])
        ),
        # rko_lio only reads node parameters from config_file when included, not from launch args
        launch_arguments={
            'config_file': LaunchConfig('lio_config'),
            'autodetect': 'false',
            'rviz': LaunchConfig('lio_rviz'),
        }.items(),
        condition=IfCondition(LaunchConfig('lio')),
    )

    # RKO-LIO reads /lidar/points_lio: the Ouster cloud minus scans with repeated or
    # backwards stamps (driver start-up), which crash it (see scripts/lidar_guard.py).
    lidar_guard = Node(
        package='sensors_bringup',
        executable='lidar_guard.py',
        name='lidar_guard',
        output='screen',
        condition=IfCondition(LaunchConfig('lio')),
    )

    base_to_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_os_sensor_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '1.08',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'lidar_frame',
        ],
    )

    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_rgb_camera_tf',
        arguments=[
            '--x', '0.08', '--y', '-0.01', '--z', '1.00',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'base_link', '--child-frame-id', 'rgb_camera_frame',
        ],
    )

    return LaunchDescription(
        [
            # must come first so every node started below picks it up
            # SetEnvironmentVariable(
            #     'FASTRTPS_DEFAULT_PROFILES_FILE',
            #     PathJoinSubstitution(
            #         [FindPackageShare('sensors_bringup'), 'config', 'fastdds_large_data.xml']
            #     ),
            # ),
            LaunchArg(
                'serial',
                default_value="'20531756'",
                description='FLIR serial number of camera (in quotes!!)',
            ),
            LaunchArg('viz', default_value='True', description='start rviz for the ouster'),
            LaunchArg('lio', default_value='true', description='start RKO-LIO odometry'),
            LaunchArg(
                'lio_config',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('sensors_bringup'), 'config', 'rko_lio.yaml']
                ),
                description='RKO-LIO parameter file',
            ),
            LaunchArg('lio_rviz', default_value='false', description='start the RKO-LIO rviz view'),
            camera_launch,
            image_rect_node,
            lidar_launch,
            lidar_guard,
            lio_launch,
            base_to_lidar_tf,
            base_to_camera_tf,
        ]
    )

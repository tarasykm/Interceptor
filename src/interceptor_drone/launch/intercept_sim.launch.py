import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('interceptor_drone'),
        'rviz', 'intercept_sim.rviz'
    )

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'],
        ),
        Node(
            package='lead_drone',
            executable='lead_drone_node',
            name='lead_drone',
            output='screen',
        ),
        Node(
            package='interceptor_drone',
            executable='interceptor_drone_node',
            name='interceptor',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])

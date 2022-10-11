from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_publisher',
            arguments = ["0", "0", "0", "0", "0", "0", "odom", "laser"],
            output = 'screen',
        ),
        Node(
            package='pose_to_tf',
            executable='pose_to_tf',
            name='pose_to_tf',
            parameters=[
                {'parent_frame': 'odom'},
                {'child_frame': 'laser'},
                {'pose_topic': 'laser_pose'},],
            output = 'screen',
        ),
    ])
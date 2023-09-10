import os
import yaml
import pathlib
import launch.actions
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='drone_pose',
            executable='DronePoseExecutable',
            name='drone_pose',
            output={
                    "stdout": "screen",
                    "stderr": "screen",
            },
            #parameters=[os.path.join(get_package_share_directory("drone_pose"), 'params', 'params.yaml')],
        )
])
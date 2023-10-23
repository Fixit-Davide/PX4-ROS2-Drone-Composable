import os
import yaml
import pathlib
import launch.actions
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

package_name = "drone_pose"

'''
Used to load parameters for composable nodes
'''
def dump_params(path, name):
    # Load the parameters specific to your ComposableNode
    with open(path, 'r') as file:
        return [yaml.safe_load(file)[name]['ros__parameters']]

def generate_launch_description():
  return LaunchDescription([
    # define static tf for base_link and camera.
    # Node(
    #     package = "tf2_ros",
    #     executable = "static_transform_publisher",
    #     arguments = ["0.12", "0.03", "0.242", "0", "0", "0", "x500_depth_0/base_link", "x500_depth_0/OakD-Lite/base_link/StereoOV7251"],
    #     output='screen',
    #     parameters=[{
    #       'use_sim_time': True}],   
    # ),
    # Node(
    #     package = "tf2_ros",
    #     executable = "static_transform_publisher",
    #     arguments = ["0.0", "0.0", "0.35", "0", "0", "0", "x500_depth_0/base_link", "x500_depth_0/lidar_link/gpu_lidar"],
    #     output='screen',
    #     parameters=[{
    #       'use_sim_time': True}], 
    # ),
    Node(
        package='drone_pose',
        executable='DronePoseExecutable',
        name='drone_pose',
        output={
                "stdout": "screen",
                "stderr": "screen",
        },
        parameters=[os.path.join(get_package_share_directory("drone_pose"), 'params', 'drone_pose.yaml')],
    ),
])
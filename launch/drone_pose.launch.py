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
    # define static tf for base_link and camera.
    # cannot use robot state publisher due to sdf files not supported.
  remappings = [('/camera', '/camera/image'),
                ('/camera_info', '/camera/camera_info'),
                ('/depth_camera', '/camera/depth_image'),
                ('depth_camera/points', '/camera/points')]
  # container = ComposableNodeContainer(
  #   name='drone_composable_node',
  #   namespace='',
  #   package='rclcpp_components',
  #   executable='component_container_mt',
  #   composable_node_descriptions=[
  #     ComposableNode(
  #         package='drone_pose',
  #         plugin='px4_autonav::DronePose',
  #         name='drone_pose',
  #         namespace='',
  #         # parameters=dump_params(os.path.join(get_package_share_directory(package_name), 'params', 'drone_pose.yaml'), "drone_pose"),
  #         extra_arguments=[{'use_intra_process_comms': True}]
  #     ),
  #     ComposableNode(
  #         package='ros_gz_bridge',
  #         plugin='ros_gz_bridge::RosGzBridge',
  #         name='ros_gz_bridge',
  #         namespace='',
  #         # arguments=[
  #         #     '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
  #         #     '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image',
  #         #     '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
  #         #     '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
  #         parameters=dump_params(os.path.join(get_package_share_directory(package_name), 'params', 'ros_gz_bridge.yaml'), "ros_gz_bridge"),
  #         extra_arguments=[{'use_intra_process_comms': True}]
  #     ),
  #   ]
  # ),
  return LaunchDescription([
    Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        arguments = ["0.12", "0.03", "0.242", "0", "0", "0", "x500_depth_0/base_link", "x500_depth_0/OakD-Lite/base_link/StereoOV7251"],
        output='screen',
    ),
    # TODO: use intraprocess communication for speed up
    Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge',
        # topic@ros_type@gazebo_type
        arguments=[
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen',
        remappings=remappings,
    ),
    Node(
        package='drone_pose',
        executable='DronePoseExecutable',
        name='drone_pose',
        output={
                "stdout": "screen",
                "stderr": "screen",
        },
        #parameters=[os.path.join(get_package_share_directory("drone_pose"), 'params', 'params.yaml')],
    ),
])
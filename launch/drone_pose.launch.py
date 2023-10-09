import os
import yaml
import pathlib
import launch.actions
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # define static tf for base_link and camera.
    # cannot use robot state publisher due to sdf files not supported.
    remappings = [('/camera', '/camera/image'),
                  ('/camera_info', '/camera/camera_info'),
                  ('/depth_camera', '/camera/depth_image'),
                  ('depth_camera/points', '/camera/points')]

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
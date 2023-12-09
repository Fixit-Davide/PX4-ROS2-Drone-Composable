# PX4-ROS2-Drone-Composable
ROS2 composable node that generates PoseStamped, Velocity and the Trajectory of the simulated PX4 UAV.  
Additionally, it generetas the tf from the `map` frame to the UAV's `base_link`.  

## Topics
|Topic|Type|Input/output|Description|
|-|-|-|-|
|vehicle_path|`nav_msgs::msg::Path`|Output|The path taken by the drone|
|vehicle_pose|`geometry_msgs::msg::PoseStamped`|Output|The current pose of the drone|
|vehicle_velocity|`visualization_msgs::msg::Marker`|Output|The current velocity vector of the drone|

## Params
|Name|Description|
|-|-|
|base_frame|The base frame's name for the tf|
|child_frame|The child frame's name for the tf|
|pub.pose|Bool to publish the pose msgs|
|pub.vel|Bool to publish the velocity vector msgs|
|pub.path|Bool to publish the path msgs|

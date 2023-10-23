/*
Copyright 2023 Lance Drone Team
*/

#ifndef DRONE__POSE__HPP_
#define DRONE__POSE__HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "message_filters/time_synchronizer.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/latest_time.h"

// px4 msgs:
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
// ROS2 msgs:
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace px4_autonav
{

class DronePose : public rclcpp::Node {
  public:

    explicit DronePose(const rclcpp::NodeOptions & options);

    ~DronePose(){};

  private:

    std::shared_ptr<nav_msgs::msg::Path> path_msg;
    bool pub_vel, pub_path, pub_pose;
    std::string base_frame, child_frame;
  
    using latest_policy = message_filters::sync_policies::LatestTime<px4_msgs::msg::VehicleAttitude,
      px4_msgs::msg::VehicleLocalPosition>;
    std::shared_ptr<message_filters::Synchronizer<latest_policy>> time_sync_;

    std::shared_ptr<message_filters::Subscriber<px4_msgs::msg::VehicleAttitude>> vehicle_attitude_sub;
    std::shared_ptr<message_filters::Subscriber<px4_msgs::msg::VehicleLocalPosition>> vehicle_local_pos_sub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vehicle_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vehicle_vel_pub_;

    void position_cb(
      const px4_msgs::msg::VehicleAttitude::ConstSharedPtr & msg_att,
      const px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr & msg_loc_pos);

  };
}  // px4_autonav

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be
// discoverable when its library is being loaded into a running process.
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(px4_autonav::DronePose)

#endif  // DRONE__POSE__HPP_

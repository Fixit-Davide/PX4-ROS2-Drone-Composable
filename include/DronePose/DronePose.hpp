/*
Copyright 2023 Lance Drone Team
*/

#ifndef DRONE__POSE__HPP_
#define DRONE__POSE__HPP_

#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include "message_filters/time_synchronizer.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/latest_time.h"

// px4 msgs:
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
// ROS2 msgs:
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace px4_autonav
{

class DronePose : public rclcpp::Node {
  public:

    DronePose(const rclcpp::NodeOptions & options);

    ~DronePose(){};

  private:

    std::shared_ptr<nav_msgs::msg::Path> path_msg;

    using latest_policy = message_filters::sync_policies::LatestTime<px4_msgs::msg::VehicleAttitude,
      px4_msgs::msg::VehicleLocalPosition>;
    std::shared_ptr<message_filters::Synchronizer<latest_policy>> time_sync_;

    std::shared_ptr<message_filters::Subscriber<px4_msgs::msg::VehicleAttitude>> vehicle_attitude_sub;
    std::shared_ptr<message_filters::Subscriber<px4_msgs::msg::VehicleLocalPosition>> vehicle_local_pos_sub;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vehicle_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vehicle_vel_pub_;

    void position_cb(
      const px4_msgs::msg::VehicleAttitude::ConstSharedPtr & msg_att,
      const px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr & msg_loc_pos);

  };
}  // px4_autonav

#endif  // DRONE__POSE__HPP_

/*
Copyright 2023 Lance Drone Team
*/

#include <iostream>
#include <chrono>
#include <cmath>
#include <string>
#include <functional>
#include <cstdlib>
#include <memory>
#include <cassert>
#include <rclcpp/qos.hpp>
#include "DronePose/DronePose.hpp"

// namespaces:
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono;
using namespace std::chrono_literals;

namespace px4_autonav{

  DronePose::DronePose(const rclcpp::NodeOptions & options)
  : Node("drone_pose", options) {
    // topics:
    rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
    vehicle_attitude_sub = std::make_shared<message_filters::Subscriber<px4_msgs::msg::VehicleAttitude>>(
                          this,
                          "/fmu/out/vehicle_attitude",
                          qos);
    vehicle_local_pos_sub = std::make_shared<message_filters::Subscriber<px4_msgs::msg::VehicleLocalPosition>>(
                          this,
                          "/fmu/out/vehicle_local_position",
                          qos);
    vehicle_path_pub_ = create_publisher<nav_msgs::msg::Path>("/vehicle_path", 10);
    vehicle_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/vehicle_pose", 10);
    vehicle_vel_pub_ = create_publisher<visualization_msgs::msg::Marker>("/vehicle_velocity", 10);
    path_msg = std::make_shared<nav_msgs::msg::Path>();
    time_sync_ = std::make_shared<message_filters::Synchronizer<latest_policy>>(
      latest_policy(this->get_clock()),
      *vehicle_attitude_sub,
      *vehicle_local_pos_sub
    );
    time_sync_->registerCallback(std::bind(&DronePose::position_cb, this, _1, _2));
    tf_static_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    RCLCPP_INFO(this->get_logger(), "Init good.");
  }

void DronePose::position_cb(
  const px4_msgs::msg::VehicleAttitude::ConstSharedPtr & msg_att,
  const px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr & msg_loc_pos) 
{
  steady_clock::time_point begin = steady_clock::now();
  geometry_msgs::msg::PoseStamped pose_msg;
  visualization_msgs::msg::Marker arrow_velocity_msg;
  rclcpp::Time time = get_clock()->now();
  pose_msg.header.frame_id = "map";
  pose_msg.header.stamp = time;
  pose_msg.pose.orientation.w = msg_att->q[0];
  pose_msg.pose.orientation.x = msg_att->q[1];
  pose_msg.pose.orientation.y = -msg_att->q[2];
  pose_msg.pose.orientation.z = -msg_att->q[3];
  pose_msg.pose.position.x = msg_loc_pos->x;
  pose_msg.pose.position.y = -msg_loc_pos->y;
  pose_msg.pose.position.z = -msg_loc_pos->z;
  vehicle_pose_pub_->publish(std::move(pose_msg));
  arrow_velocity_msg.action = 0;
  arrow_velocity_msg.header.frame_id = "map";
  arrow_velocity_msg.header.stamp = time;
  arrow_velocity_msg.ns = "arrow";
  arrow_velocity_msg.type = 0;
  arrow_velocity_msg.scale.x = 0.1;
  arrow_velocity_msg.scale.y = 0.2;
  arrow_velocity_msg.scale.z = 0.1;
  arrow_velocity_msg.color.r = 0.5;
  arrow_velocity_msg.color.g = 0.5;
  arrow_velocity_msg.color.b = 0.0;
  arrow_velocity_msg.color.a = 1.0;
  float dt = 0.2;
  geometry_msgs::msg::Point tail, head;
  tail.x = msg_loc_pos->x;
  tail.y = -msg_loc_pos->y;
  tail.z = -msg_loc_pos->z;
  head.x = msg_loc_pos->x + dt * msg_loc_pos->vx;
  head.y = -msg_loc_pos->y + dt * -msg_loc_pos->vy;
  head.z = -msg_loc_pos->z + dt * -msg_loc_pos->vz;
  arrow_velocity_msg.points = {tail, head};
  vehicle_vel_pub_->publish(std::move(arrow_velocity_msg));

  path_msg->header = pose_msg.header;
  path_msg->poses.push_back(pose_msg);
  vehicle_path_pub_->publish(*path_msg);

  // tf broadcaster: base_link of the drone
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = time;
  tf.header.frame_id = "map";
  tf.child_frame_id = "x500_depth_0/base_link";
  tf.transform.translation.x = msg_loc_pos->x;
  tf.transform.translation.y = -msg_loc_pos->y;
  tf.transform.translation.z = -msg_loc_pos->z;
  tf.transform.rotation.x = msg_att->q[1];
  tf.transform.rotation.y = -msg_att->q[2];
  tf.transform.rotation.z = -msg_att->q[3];
  tf.transform.rotation.w = msg_att->q[0];
  tf_static_broadcaster_->sendTransform(tf);
  steady_clock::time_point end = steady_clock::now();
  // RCLCPP_INFO(
  //   get_logger(), "[Time passed] = %ld [ms]",
  //   duration_cast<milliseconds>(end - begin).count());
}

}  // px4_autonav
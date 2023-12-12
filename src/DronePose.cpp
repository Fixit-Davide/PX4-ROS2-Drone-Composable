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
using namespace px4_msgs::msg;
// using std::placeholders::_2;
using namespace std::chrono;
using namespace std::chrono_literals;

namespace px4_autonav{

  DronePose::DronePose(const rclcpp::NodeOptions & options)
  : Node("drone_pose", options) {
    // parameters:
    base_frame = declare_parameter<std::string>("base_frame", "/map");
    child_frame = declare_parameter<std::string>("child_frame", "/base_link");
    pub_pose = declare_parameter<bool>("pub.pose", false);
    pub_vel = declare_parameter<bool>("pub.vel", false);
    pub_path = declare_parameter<bool>("pub.path", false);
    // topics:
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5), rmw_qos_profile_sensor_data);
    vehicle_odom_sub_ = create_subscription<VehicleOdometry>(
                      "/fmu/out/vehicle_odometry",
                      qos,
                      std::bind(&DronePose::position_cb, this, _1));
    vehicle_path_pub_ = create_publisher<nav_msgs::msg::Path>("/vehicle_path", 10);
    vehicle_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/vehicle_pose", 10);
    vehicle_vel_pub_ = create_publisher<visualization_msgs::msg::Marker>("/vehicle_velocity", 10);
    path_msg = std::make_shared<nav_msgs::msg::Path>();
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    RCLCPP_INFO(this->get_logger(), "Init good.");
  }

void DronePose::position_cb(
  const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg) 
{
  steady_clock::time_point begin = steady_clock::now();
  geometry_msgs::msg::PoseStamped pose_msg;
  visualization_msgs::msg::Marker arrow_velocity_msg;
  rclcpp::Time time = get_clock()->now();
  if(pub_pose){
    pose_msg.header.frame_id = base_frame;
    pose_msg.header.stamp = time;
    pose_msg.pose.orientation.w = odom_msg->q[0];
    pose_msg.pose.orientation.x = odom_msg->q[1];
    pose_msg.pose.orientation.y = -odom_msg->q[2];
    pose_msg.pose.orientation.z = -odom_msg->q[3];
    pose_msg.pose.position.x = odom_msg->position[0];
    pose_msg.pose.position.y = -odom_msg->position[1];
    pose_msg.pose.position.z = -odom_msg->position[2];
    vehicle_pose_pub_->publish(std::move(pose_msg));
  }
  if(pub_vel){
    arrow_velocity_msg.action = 0;
    arrow_velocity_msg.header.frame_id = base_frame;
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
    tail.x = odom_msg->position[0];
    tail.y = -odom_msg->position[1];
    tail.z = -odom_msg->position[2];
    head.x = odom_msg->position[0] + dt * odom_msg->velocity[0];
    head.y = -odom_msg->position[1] + dt * -odom_msg->velocity[1];
    head.z = -odom_msg->position[2] + dt * -odom_msg->velocity[2];
    arrow_velocity_msg.points = {tail, head};
    vehicle_vel_pub_->publish(std::move(arrow_velocity_msg));
  }
  if(pub_path){
    path_msg->header = pose_msg.header;
    path_msg->poses.push_back(pose_msg);
    vehicle_path_pub_->publish(*path_msg);
  }

  // tf broadcaster: base_link of the drone
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = time;
  tf.header.frame_id = base_frame;
  tf.child_frame_id = child_frame;
  tf.transform.translation.x = odom_msg->position[0];
  tf.transform.translation.y = -odom_msg->position[1];
  tf.transform.translation.z = -odom_msg->position[2];
  tf.transform.rotation.x = odom_msg->q[1];
  tf.transform.rotation.y = -odom_msg->q[2];
  tf.transform.rotation.z = -odom_msg->q[3];
  tf.transform.rotation.w = odom_msg->q[0];
  tf_broadcaster_->sendTransform(tf);
  steady_clock::time_point end = steady_clock::now();
  // RCLCPP_INFO(
  //   get_logger(), "[Time passed] = %ld [ms]",
  //   duration_cast<milliseconds>(end - begin).count());
}

}  // px4_autonav
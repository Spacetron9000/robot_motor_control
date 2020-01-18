#include "wheel_odometry/converter.h"
#include "ros/ros.h"
#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <thread>
#include <memory>
#include <mutex>

EncoderConverter::EncoderConverter() : nh_("~")
{
  std::cout<<"In the constructor"<<std::endl;
  left_sub_ = nh_.subscribe("/encoders/left",1, &EncoderConverter::left_tick_cb, this);
  right_sub_ = nh_.subscribe("/encoders/right",1, &EncoderConverter::right_tick_cb, this);
  odom_publisher_ = nh_.advertise<nav_msgs::Odometry>("/odom", 1000);
  run_odom_calc_thread_ = true;
  odom_calc_thread_ = std::thread(&EncoderConverter::calc_odometry_from_ticks, this);
  curr_time_ = ros::Time::now();

  while(ros::ok()) {
    ros::spinOnce();
  }
}

EncoderConverter::~EncoderConverter() {
  run_odom_calc_thread_ = false;
  odom_calc_thread_.join();
}

void EncoderConverter::left_tick_cb(const std_msgs::Int16 &data) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  prev_left_ticks_ = left_ticks_;
  left_ticks_ = data.data;
}
void EncoderConverter::right_tick_cb(const std_msgs::Int16 &data) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  prev_right_ticks_ = right_ticks_;
  right_ticks_ = data.data;
}

void EncoderConverter::calc_odometry_from_ticks() {
  ros::Rate r(100);
  while(run_odom_calc_thread_) {
    prev_time_ = curr_time_;
    curr_time_ = ros::Time::now();
    double dt = (curr_time_ - prev_time_).toSec();

    if (prev_time_.toSec() > 0.0 && dt > 0.0) {

      delta_left_ = left_ticks_ - prev_left_ticks_;
      delta_right_ = right_ticks_ - prev_right_ticks_;

      v_left_ = (delta_left_ * kDistPerCount_) / dt;
      v_right_ = (delta_right_ * kDistPerCount_) / dt;
      v_center_ = (v_left_ + v_right_)/2.0;

      vth_ = (v_right_ - v_left_) / 2*kLengthBetweenWheels_;
      vx_ = v_center_ * cos(th_ + vth_/2);
      vy_ = v_center_ * sin(th_ + vth_/2);

      x_+=vx_*dt;
      y_+=vy_*dt;
      th_+=vth_*dt;

      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);

      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = curr_time_;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = x_;
      odom_trans.transform.translation.y = y_;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;
      odom_broadcaster_.sendTransform(odom_trans);


      odom_.header.stamp = curr_time_;
      odom_.header.frame_id = "odom";

      odom_.pose.pose.position.x = x_;
      odom_.pose.pose.position.y = y_;
      odom_.pose.pose.position.z = 0.0;
      odom_.pose.pose.orientation = odom_quat;

      //TODO: Tune the covariance matrix
      odom_.pose.covariance = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

      odom_.child_frame_id = "base_link";
      odom_.twist.twist.linear.x = vx_;
      odom_.twist.twist.linear.y = vy_;
      odom_.twist.twist.angular.z = vth_;
      odom_publisher_.publish(odom_);
    }


    r.sleep();
  }
}

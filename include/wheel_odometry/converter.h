#include "ros/ros.h"
#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <thread>
#include <memory>
#include <mutex>

#pragma once

class EncoderConverter {

public:

  EncoderConverter();

  ~EncoderConverter();

  void left_tick_cb(const std_msgs::Int16 &data);
  void right_tick_cb(const std_msgs::Int16 &data);
  void calc_odometry_from_ticks();

private:
  int left_ticks_ {0};
  int right_ticks_ {0};
  int prev_left_ticks_ {0};
  int prev_right_ticks_ {0};

  int delta_left_ {0};
  int delta_right_ {0};

  double v_left_ {0.};
  double v_right_ {0.};
  double v_center_ {0.};

  double x_{0.};
  double y_ {0.};
  double th_ {0.};

  double vx_ {0.};
  double vy_ {0.};
  double vth_ {0.};

  nav_msgs::Odometry odom_;

  ros::Time curr_time_;
  ros::Time prev_time_;

  const double kDistPerCount_ {(3.14159265 * 0.065) / 20.};
  const double kLengthBetweenWheels_{0.126};

  ros::NodeHandle nh_;
  ros::Publisher odom_publisher_;
  tf::TransformBroadcaster odom_broadcaster_;
  ros::Subscriber left_sub_;
  ros::Subscriber right_sub_;
  std::thread odom_calc_thread_;
  std::mutex data_mutex_;
  bool run_odom_calc_thread_{false};
};

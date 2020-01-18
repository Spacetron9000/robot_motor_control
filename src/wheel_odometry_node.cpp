#include "wheel_odometry/converter.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry");
  EncoderConverter *converter = new EncoderConverter();

}

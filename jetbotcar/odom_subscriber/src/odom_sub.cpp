#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

void counterCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  // ROS_INFO("%s", msg->header.frame_id.c_str());
  // ROS_INFO("%f", msg->twist.twist.linear.x);
  ROS_INFO("%f", msg->pose.pose.position.x);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "odom_sub_node");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("odom", 1000, counterCallback);

  ros::spin();

  return 0;
}
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include "ros/ros.h"
//#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmd_vel");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);   
  ros::Rate loop_rate(10);

 int count = 0;
  while (ros::ok())
  {
    geometry_msgs::Twist msg;
    msg.linear.x = 10;
    msg.angular.z = 5;    
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
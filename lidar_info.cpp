#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Pose2D.h>
#include <sstream>

float r;
float theta;

void r_theta_value(const geometry_msgs::Pose2D::ConstPtr& msg)
{
 r=msg->x;
 theta=msg->theta;
 return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_info");
  ros::NodeHandle n;
  ros::Publisher lidar_pub = n.advertise<geometry_msgs::Pose2D>("/lidar", 1000);
  ros::Subscriber r_theta_value_sub = n.subscribe("r_theta_value", 100, r_theta_value);// subscriber to get MAV position
  float obs_theta;
  ros::Rate loop_rate(10);//theta and r are given by motor and lidar
  int count = 0,r_min;
  while (ros::ok())
  {
    
    geometry_msgs::Pose2D msg1;
    while(theta != 0 && count!=0)
    {
     count=1;
     r_min=999;
     if(r<r_min)
     {
     r_min=r;
     obs_theta=theta;
     }
    }
    msg1.x=r_min;
    msg1.theta=obs_theta;
    lidar_pub.publish(msg1);
    count=0;
    ros::spinOnce();
    loop_rate.sleep();
  }
return 0;
}




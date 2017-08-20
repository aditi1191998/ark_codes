#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Pose2D.h>
#include <sstream>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_info");
  ros::NodeHandle n;
  ros::Publisher lidar_pub = n.advertise<geometry_msgs::Pose2D>("lidar", 1000);

  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    geometry_msgs::Pose2D msg;
    msg.x=1;
    msg.theta=90;
    lidar_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

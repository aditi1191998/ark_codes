
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
turtlesim::Pose Curpose;
ros::Publisher velocity_publisher;
geometry_msgs::Twist vel; 
void CurposeCallback(const turtlesim::Pose::ConstPtr& msg);
float GetErrorAng(turtlesim::Pose Curpose);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "random");        
    ros::NodeHandle n; 
    ros::Subscriber Curpose_sub = n.subscribe("/turtle2/pose", 5, CurposeCallback);
    velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel",10);
    ros::Rate loop_rate(10);  
       
    while (ros::ok()){
       ros::spinOnce();
      float ErrorAng = GetErrorAng(Curpose);
      if((2.1<=Curpose.x && Curpose.x<2.2))
      {
        vel.angular.z = 0; 
        vel.linear.x = 0;
      }
      else
      {
       vel.angular.z = 10 * ErrorAng; 
      vel.linear.x = 1;
      printf("%f\n",Curpose.x);
     
      }
       velocity_publisher.publish(vel);
      loop_rate.sleep();
    }
}

void CurposeCallback(const turtlesim::Pose::ConstPtr& msg)          
{
    Curpose.x = msg->x;
    Curpose.y = msg->y;
    Curpose.theta = msg->theta;                                        
    return;
}

float GetErrorAng(turtlesim::Pose Curpose)
{
    float Ex = 2.0 - Curpose.x;                                    
    float Ey = 2.0 - Curpose.y;                                   

    // get desire angle
    float dest = atan2f(Ey, Ex);                                       

    // get angle error
    float Et = dest - Curpose.theta;

    
    return (Et);
}





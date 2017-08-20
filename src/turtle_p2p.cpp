#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <math.h>

geometry_msgs::Twist move;
turtlesim::Pose obstacle, goal;
bool avoid;
float set_theta, obs_theta;
float x,y,z;
float errorA=0,preverrorA=0,IA=0;
float errorB=0,preverrorB=0,IB=0;
float errorC=0,preverrorC=0,IC=0;
const float kpA=5.5;
const float kiA=0.001;
const float kdA=0;
const float kpB=0.5;
const float kiB=0;
const float kdB=0;
const float kpC=0.6;
const float kiC=0;
const float kdC=0;

void poseCallback(const turtlesim::Pose::ConstPtr& pose)
{

  if(sqrt(pow(obstacle.x-pose->x,2)+pow(obstacle.y-pose->y,2)) <= 2)
    avoid = true;
  else
    avoid = false;

  set_theta = atan((goal.y-pose->y)/(goal.x-pose->x));
  if(set_theta<0)
    if(goal.y>pose->y)
      set_theta+=M_PI;
    else
      set_theta+=2*M_PI;
  else
    if(goal.y<pose->y)
      set_theta+=M_PI;

  obs_theta = atan((obstacle.y-pose->y)/(obstacle.x-pose->x));
  if(obs_theta<0)
    if(obstacle.y>pose->y)
      obs_theta+=M_PI;
    else
      obs_theta+=2*M_PI;
  else
    if(obstacle.y<pose->y)
      obs_theta+=M_PI;

  if(avoid){
    errorC = sqrt(pow(obstacle.x-pose->x,2)+pow(obstacle.y-pose->y,2));
    IC+=errorC;
    z = kpC*errorC + kiC*IC + kdC*(errorC-preverrorC);
    preverrorC=errorC;
    ROS_WARN("obstacle %lf", sqrt(pow(obstacle.x-pose->x,2)+pow(obstacle.y-pose->y,2)));
    if(set_theta < obs_theta)
      set_theta-=z;
    else
      set_theta+=z;
  }

  errorA = set_theta - pose->theta;
  IA+=errorA;
  x = kpA*errorA + kiA*IA + kdA*(errorA-preverrorA);
  if(x<-1)
    x = -1;
  else if(x>1)
    x = 1;
  preverrorA=errorA;

  errorB = sqrt(pow(goal.x-pose->x,2)+pow(goal.y-pose->y,2));
  IB+=errorB;
  y = kpB*errorB + kiB*IB + kdB*(errorB-preverrorB);
  if(y<-1)
    y = -1;
  else if(y>2)
    y = 2;
  preverrorB=errorB;

  move.angular.z = x;
  move.linear.x = y;
}

void poseCallback2(const turtlesim::Pose::ConstPtr& pose2){
obstacle.x=pose2->x;
obstacle.y=pose2->y;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_p2p");
    ros::NodeHandle n;
    ros::Subscriber sub_tur = n.subscribe<turtlesim::Pose>("/turtle1/pose", 1, poseCallback);
    ros::Subscriber sub_tur2 = n.subscribe<turtlesim::Pose>("/turtle2/pose", 1, poseCallback2);
    ros::Publisher pub_tur = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
    ros::Rate loop_rate(10);
    goal.x=9.0;
    goal.y=9.0;
    while (ros::ok()){
      pub_tur.publish(move);
      ros::spinOnce();
      loop_rate.sleep();
    }

    return 0;
}

#include "ros/ros.h"
#include "ardrone_autonomy/Navdata.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "math.h"

geometry_msgs::Twist vel;

int E=0,Eold=0,d,e,dv=800;
float ref,kp,ki,kd;

int Ey=0,Eoldy=0,dy,ey,dvy=80;
float refy,kpy,kiy,kdy;

int rad=0,theta=0;
const int pi=3.14;
float velx,vely,cirx,ciry,cirX=0,cirY=0,xold=0,yold=0,dcx,dcy;
float px=1,py=1,ix,iy,dlx,dly,dvr=80;

void pid(const ardrone_autonomy::Navdata::ConstPtr& msg){
  int alt=msg->altd;
  e=ref-alt;
  E+=e;
  d=e-Eold;

  int yaw=msg->rotZ;
  ey=refy-yaw;
  Ey+=ey;
  dy=ey-Eoldy;

  vel.linear.z = (kp*e + ki*E +kd*d)/dv;
  vel.angular.z = (kpy*ey + kiy*Ey +kdy*dy)/dvy;

  Eold = e;
  Eoldy=e;

  if(rad){
    theta= atan((msg->vy)/(msg->vx));

    velx = -1*rad*sin(theta*2*pi/360);
    vely =  rad*cos(theta*2*pi/360);

    cirx = velx-(msg->vx);
    ciry = vely-(msg->vy);

    cirX+=cirx;
    cirY+=ciry;
    dlx=cirx-xold;
    dly=ciry-yold;

    vel.linear.x = (px*cirx + ix*cirX + dlx*dcx)/dvr;
    vel.linear.y = (py*ciry + iy*cirY + dly*dcy)/dvr;
    }

}


int main(int argc, char **argv){
  ros::init(argc, argv, "althold");
  ros::NodeHandle n;

  ros::Publisher  takeoff=n.advertise<std_msgs::Empty>("/ardrone/takeoff",10,true);
  ros::Publisher  land=n.advertise<std_msgs::Empty>("/ardrone/land",10,true);

  std_msgs::Empty blank;

  if(argc==3){
    ref = std::atof(argv[1]);
    refy = std::atof(argv[2]);
    kp=0.55;
    kd=0.25;
    kpy=1.5;
  }
  else if(std::atof(argv[5])==1){
    ref = std::atof(argv[1]);
    kp = std::atof(argv[2]);
    ki = std::atof(argv[3]);
    kd = std::atof(argv[4]);
  }
  else if(std::atof(argv[5])==2){
    refy = std::atof(argv[1]);
    kpy = std::atof(argv[2]);
    kiy = std::atof(argv[3]);
    kdy = std::atof(argv[4]);
  }
  else if(argc==9){
    ref = std::atof(argv[1]);
    kp = std::atof(argv[2]);
    ki = std::atof(argv[3]);
    kd = std::atof(argv[4]);
    refy = std::atof(argv[5]);
    kpy = std::atof(argv[6]);
    kiy = std::atof(argv[7]);
    kdy = std::atof(argv[8]);
  }
  else if(argc==2){
    rad = std::atof(argv[1]);
    ref = 1000;
    refy = 0;
    kp=0.55;
    kd=0.25;
    kpy=1.5;
  }
  else
    land.publish(blank);

  ros::Subscriber sub = n.subscribe("/ardrone/navdata", 1000, pid);
  ros::Publisher  pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  ros::Rate loop_rate(10);
  takeoff.publish(blank);

  while (ros::ok()){
    pub.publish(vel);
    ROS_INFO("%lf %d    %lf %d\n",vel.linear.z,e,vel.angular.z,ey);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

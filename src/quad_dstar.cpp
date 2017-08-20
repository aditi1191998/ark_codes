#include "ark_llp/go2goal.h"
#include "ardrone_autonomy/Navdata.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <math.h>
#define Default 2.5
nav_msgs::Odometry obspose;
nav_msgs::Odometry quadpose;
void obsCallback(const nav_msgs::Odometry::ConstPtr& msg);
void quadCallback(const nav_msgs::Odometry::ConstPtr& msg);
int flago=0;
int flagq=0;

int k=1;
struct coordinates{
int x;
int y;
};
typedef struct coordinates cd;

struct node{
cd loc;
cd parent;
int g;            //cost function
int h;            //heuristic
int f;
int flag;            //total
};

typedef struct node node;


int check(int x,int y,node list[1000],int size);
node* avoid(int xs,int ys,int xg,int yg,int xo,int yo);
int min(node list[1000],int size);

int main(int argc, char **argv)
{
  ros::init(argc,argv,"quad_dstar");
  Go2Goal dest;
  node* path_final;
  int xs,ys,xg,yg;
  int count=0;

  xg = std::atof(argv[1]);//final point
  yg = std::atof(argv[2]);
  ros::NodeHandle n;
  ros::Subscriber quadpose_sub = n.subscribe("/ground_truth/state", 100, quadCallback); // subscriber to get MAV position
  ros::Subscriber obspose_sub = n.subscribe("/robot3/odom", 100, obsCallback); // subscriber to get ground bot position
  ros::Rate loop_rate(1);

  int a=0;

  while(ros::ok())
   {
    if(flagq==1 && flago==1)//to ensure we get correct position of obstacle and quad
    {
    int xs=roundf((quadpose.pose.pose.position.y)*(-1));
    int ys=roundf(quadpose.pose.pose.position.x);
    int xo=roundf((obspose.pose.pose.position.y)*(-1));
    int yo=roundf(obspose.pose.pose.position.x);
    float dis=sqrt((xo-xg)*(xo-xg)+(yo-yg)*(yo-yg));//if obstacle on the goal
    if( dis == 0)
    {
      ROS_INFO("%f",dis);
      dest.set_dest((quadpose.pose.pose.position.y)*(-1),quadpose.pose.pose.position.x,2.5,0);//hover till obstacle passes goal
    }
    else
    {
     path_final=avoid(xs,ys,xg,yg,xo,yo);//generates path
    if(xs==xg && ys==yg) //reached goal
    {
      break;
    }
    ROS_INFO("%d %d %d %d",xs,ys,xo,yo);
    for(a=0;a<k;a++)
    {
    ROS_INFO("%d %d",path_final[k-1-a].loc.x,path_final[k-1-a].loc.y);
    }
     dest.set_dest(path_final[k-1].loc.x,path_final[k-1].loc.y,2,0);//clears queue for new path generation after each iteration
  for(a=1;a<k;a++)
 {
   dest.add_dest(path_final[k-1-a].loc.x,path_final[k-1-a].loc.y,2,0);
 }
}
}
   ros::spinOnce();
   loop_rate.sleep();

}

}
//same as astar
node* avoid(int xs,int ys,int xg,int yg,int xo,int yo)//xo and yo are position of obstacles
{
  node open_l[1000];
  node close_l[1000];
  node path[1000];
  int size_c=0;
  int i=0,j=0,size_o=0,index;
  open_l[i].loc.x=xs;
  open_l[i].loc.y=ys;
  open_l[i].parent.x=999;
  open_l[i].parent.y=999;
  open_l[i].g=0;
  open_l[i].h=0;
  open_l[i].flag=0;
  open_l[i].f=open_l[i].g+open_l[i].h;
  size_o++;
  node temp;
  node top=open_l[0];                  //max priority element

  while(!(top.loc.x==xg && top.loc.y==yg))
  {
  for(i=-1;i<2;i++)
  {
    for(j=-1;j<2;j++)
    {
      if(!(i==0 && j==0))
      {
        if(top.loc.x+i<=10 && top.loc.x+i>=-10 && top.loc.y+j<=10 && top.loc.y+j>=-10 && !(top.loc.x+i==xo && top.loc.y+j==yo))
        {
        if(check(top.loc.x+i,top.loc.y+j,close_l,size_c) != 999)
        {
          //do nothing
        }
        else
        {
          if(i==0||j==0)
          temp.g=top.g + 10;
          else
          temp.g=top.g + 14;
          temp.h=(abs(xg-(top.loc.x+i))+abs(yg-(top.loc.y+j)))*10;
          temp.f=temp.g+temp.h;
          temp.parent.x=top.loc.x;
          temp.parent.y=top.loc.y;
          if(check(top.loc.x+i,top.loc.y+j,open_l,size_o) == 999)
          {
            open_l[size_o].loc.x=top.loc.x+i;
            open_l[size_o].loc.y=top.loc.y+j;
            open_l[size_o].parent.x=temp.parent.x;
            open_l[size_o].parent.y=temp.parent.y;
            open_l[size_o].g=temp.g;
            open_l[size_o].h=temp.h;
            open_l[size_o].f=temp.f;
            open_l[size_o].flag=1;
            size_o++;
          }
          else
          {
            index=check(top.loc.x+i,top.loc.y+j,open_l,size_o);
            if(open_l[index].f >= temp.f)
            {
              open_l[index].f=temp.f;
              open_l[index].parent.x=temp.parent.x;
              open_l[index].parent.y=temp.parent.y;
            }
          }
        }
      }
    }
    }
   }
        index=min(open_l,size_o);

      open_l[index].flag=0;
      close_l[size_c]=open_l[index];
      close_l[size_c].flag=1;
      size_c++;
      top=open_l[index];

  }

    path[0]=close_l[size_c-1];
    int size=size_c-1;
    k=1;
    while(size > 0)
    {
      if(close_l[size].parent.x == close_l[size-1].loc.x && close_l[size].parent.y == close_l[size-1].loc.y )
      {
        path[k]=close_l[size-1];
        k++;
      }
      size--;
    }
    return path;

   }

  int check(int x,int y,node list[1000],int size)
  {
    int i;
    for(i=0;i<size ;i++)
    {

      if(x == list[i].loc.x && y == list[i].loc.y && list[i].flag != 0)            //if in list
      {
        return(i);
      }
    }
    return(999);                                      //not in list
  }


  int min(node list[1000],int size)
  {
    int i,index;
    node temp;
    temp.f=500;
    for(i=0;i<size;i++)
    {

      if((list[i].f < temp.f) && list[i].flag!=0)
      {
        temp=list[i];
        index=i;
      }
    }
    return index;
  }

  void obsCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    flago=1;
    obspose.pose.pose.position.x = msg->pose.pose.position.x;
    obspose.pose.pose.position.y = msg->pose.pose.position.y;
    obspose.pose.pose.position.z = msg->pose.pose.position.z;
    return;
  }
  void quadCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    flagq=1;
    quadpose.pose.pose.position.x = msg->pose.pose.position.x;
    quadpose.pose.pose.position.y = msg->pose.pose.position.y;
    quadpose.pose.pose.position.z = msg->pose.pose.position.z;
    return;
  }

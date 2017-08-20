#include "ark_llp/go2goal.h"
#include <ros/ros.h>
#include <sstream>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "quad_p2p");
    Go2Goal random; 
    double x, y, z, theta;
    float func;
    x = std::atof(argv[1]);
    y = std::atof(argv[2]);
    z = std::atof(argv[3]);
    theta = std::atof(argv[4]);
    func = std::atof(argv[5]);
    //double x=1,y=2,z=3,theta=0;
    //double x1=0.1,y1=0.2,z1=3,theta1=0;
    if(func==1)
    	random.set_dest(x, y, z, theta);
    else
    	random.false_dest(x, y, z, theta);
    return 0;
}

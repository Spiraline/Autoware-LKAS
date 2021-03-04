#include <ros/ros.h>
#include <iostream>
#include "pcd_clipper.h"

int main(int argc, char* argv[]){
    ros::init(argc, argv, "pcd_clipper");
    PCDClipper clipper;
    clipper.MainLoop();    

    return 0;

}
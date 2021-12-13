#include "ros/ros.h"
#include <vector>
#include "eigen_icp_2d.h"
#include "Eigen/Geometry"
#include "Eigen/Cholesky"
#include "sensor_msgs/LaserScan.h"
#include "rotations.h"
#include <iostream>
#include <fstream>

using Vector2fVector=std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>;
using ContainerType=ICP::ContainerType;

int first_msg;
ContainerType old;
ContainerType niu;
float sample_num; //to get a sample every sample_num 
Eigen::Isometry2f T0; //current transformation of read points
//need to do a structure to keep the isometry updated and the set of actual points to compute the relative trasformation
void matcher_cb(const sensor_msgs::LaserScan &scan) {
  
  ContainerType& cur = niu;
  float angle_min = scan.angle_min;       
  float angle_max = scan.angle_max; 
  float angle_increment = scan.angle_increment;
  float size = (angle_max-angle_min)/angle_increment/sample_num;
  
  if(!first_msg){
    first_msg=1; 
    old.reserve(size);
    niu.reserve(size);
    cur = old; 
  }
  float line;
  float angle;
  for(int i=0; i<size; i+=sample_num){
    line = scan.ranges[i];
    angle = angle_min+angle_increment*i;
    cur[i] = Eigen::Vector2f(line*cos(angle), line*sin(angle));
  }
  if(first_msg==0) return;
  ICP icp(old, cur, 10);
  icp.run(100);

  // std::cerr << T0.translation() << std::endl;
}

int main(int argc, char **argv) {  
  sample_num=1;
  T0 = Eigen::Isometry2f::Identity();
  // std::cerr << T0.matrix() << std::endl;
  ros::init(argc, argv, "laser_matcher");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  ros::Subscriber sub_lm = n.subscribe("/base_scan", 1000, matcher_cb);
  ros::spin();
}
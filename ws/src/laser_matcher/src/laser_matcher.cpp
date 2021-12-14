#include "ros/ros.h"
#include <vector>
#include <iostream>
#include <fstream>
#include "eigen_icp_2d.h"
#include "Eigen/Geometry"
#include "Eigen/Cholesky"
#include "sensor_msgs/LaserScan.h"
#include "rotations.h"

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

  std::cerr << size << std::endl;
  std::cerr << angle_min << std::endl;
  std::cerr << angle_max << std::endl;
  std::cerr << angle_increment << std::endl;
  
  if(first_msg==0){
    first_msg=1; 
    old.reserve(size);
    niu.reserve(size);
  }
  float line;
  float angle=angle_min;
  for(int i=0; i<size; i+=1){
    line = scan.ranges[i];
    angle += angle_increment*sample_num;
    if(first_msg==0)
      old[i] = T0*Eigen::Vector2f(line*cos(angle), line*sin(angle));
    else
      niu[i] = T0*Eigen::Vector2f(line*cos(angle), line*sin(angle));
    // std::cerr << cur[i] << std::endl;
  }
  
  ICP icp(old, niu, 10);
  icp.run(1);
  T0=T0*icp.X();
  std::cout << icp.X().matrix() << std::endl;
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

  // for(int i=0; i < 360; i++)
  //   std::cerr << old[i]-niu[i] << std::endl;
}
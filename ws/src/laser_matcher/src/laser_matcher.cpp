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

int num_msg=0;
ContainerType old(360);
ContainerType niu(360);
float sample_num; //to get a sample every sample_num 
Eigen::Isometry2f T0; //current transformation of read points
//need to do a structure to keep the isometry updated and the set of actual points to compute the relative trasformation
void matcher_cb(const sensor_msgs::LaserScan &scan) {
  
  ContainerType& cur = niu;
  float angle_min = scan.angle_min;       
  float angle_max = scan.angle_max; 
  float angle_increment = scan.angle_increment;
  float size = (angle_max-angle_min)/angle_increment/sample_num;

  // std::cerr << size << std::endl;
  // std::cerr << angle_min << std::endl;
  // std::cerr << angle_max << std::endl;
  // std::cerr << angle_increment << std::endl;
  std::cerr << "msg num" << num_msg << std::endl;
  // if(num_msg==0){
  //   old.reserve(size);
  //   niu.reserve(size);
  // }
  float line;
  float angle=angle_min;
  for(int i=0; i<size; i+=1){
    line = scan.ranges[i*sample_num];
    angle += angle_increment*sample_num;
    //if line > scan.maxrange etc
    float a = line*cos(angle);
    float b = line*sin(angle);
    if(num_msg==0)
      old[i] = Eigen::Vector2f(a,b);
    else
      niu[i] = Eigen::Vector2f(a,b);
      // std::cerr << "old " << old[i](1) << " " << old[i](2) << std::endl;
      // std::cerr << "niu " << a << " " << b << std::endl;
    // std::cerr << cur[i] << std::endl;
    

  }
   
  if(num_msg!=0){
    ICP icp(old, niu, 10);
    icp.run(5);
    T0=T0*icp.X();
    std::cerr << "Origin position" << std::endl;
    std::cerr << T0.translation() << std::endl;
    old=niu;
  }
  num_msg++;
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
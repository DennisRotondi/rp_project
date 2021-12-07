#include "ros/ros.h"
#include <vector>
#include "eigen_icp_2d.h"
#include "Eigen/Geometry"
#include "Eigen/Cholesky"
#include "sensor_msgs/LaserScan.h"
#include "rotations.h"
#include <iostream>
#include <fstream>

using Vector2fVector=std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> >;
using ContainerType=ICP::ContainerType;

void matcher_cb(const sensor_msgs::LaserScan &scan) {
    std::cerr << "wew " << std::endl;
    ContainerType kd_points(100);
    ContainerType transformed_points(100);
    ICP icp(kd_points, transformed_points, 10);
    icp.run(100);
}

int main(int argc, char **argv) {  
  ros::init(argc, argv, "laser_matcher");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  ros::Subscriber sub_lm = n.subscribe("/scan", 1000, matcher_cb);
  ros::spin();
}
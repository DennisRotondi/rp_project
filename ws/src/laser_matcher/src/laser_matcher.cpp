#include "ros/ros.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <memory>
#include "eigen_laserm_2d.h"
#include "Eigen/Geometry"
#include "Eigen/Cholesky"
#include "sensor_msgs/LaserScan.h"
#include "rotations.h"

// using Vector2fVector=std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>;
using ContainerType=LASERM::ContainerType;

using std::cerr; using std::endl;

int num_msg=0;
std::unique_ptr<LASERM> laser_matcher;
float sample_num; //to get a sample every sample_num 
void matcher_cb(const sensor_msgs::LaserScan &scan) {
  
  float angle_min = scan.angle_min;       
  float angle_max = scan.angle_max; 
  float angle_increment = scan.angle_increment;
  int size = std::ceil((angle_max-angle_min)/angle_increment/sample_num);
  std::cerr << size << std::endl;
  // std::cerr << angle_min << std::endl;
  // std::cerr << angle_max << std::endl;
  // std::cerr << angle_increment << std::endl;

  cerr << "msg num" << num_msg << endl;
  if(num_msg==0) laser_matcher=std::unique_ptr<LASERM>(new LASERM(size,10));
  if(num_msg>1) laser_matcher->updateOld();
  int id = num_msg!=0;

  float line;
  float angle=angle_min;
  int idx=0;
  for(int i=0; i<size; i+=1){
    line = scan.ranges[i*sample_num];
    angle += angle_increment*sample_num;
    if (line > scan.range_max || line < scan.range_min){
      // cerr << "values < range_min or > range_max should be discarded" << endl;
      size--;
    }
    idx++;
    float a = line*cos(angle);
    float b = line*sin(angle);
    laser_matcher->setSet(id,idx, Eigen::Vector2f(a,b));
    // std::cerr << "old " << old[i](1) << " " << old[i](2) << std::endl;
    // std::cerr << "niu " << a << " " << b << std::endl;
    // std::cerr << cur[i] << std::endl;
  }
  
  num_msg++;
  if(num_msg==1) return;
  if(size!=laser_matcher->niu().size()){
    laser_matcher->resizeNiu(size);
  }
  laser_matcher->run(2);
  laser_matcher->updateTB();
  //update with publisher of pose 2d
  
}

int main(int argc, char **argv) {  
  sample_num=1;
  ros::init(argc, argv, "laser_matcher");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  ros::Subscriber sub_lm = n.subscribe("/base_scan", 1000, matcher_cb);
  ros::spin();

  // for(int i=0; i < 360; i++)
  //   std::cerr << old[i]-niu[i] << std::endl;
}
#include "ros/ros.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <memory>
#include "eigen_laserm_2d.h"
#include "Eigen/Geometry"
#include "tf/tf.h"
#include "tf2_msgs/TFMessage.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "Eigen/Cholesky"
#include "sensor_msgs/LaserScan.h"
#include "rotations.h"
#include <geometry_msgs/Pose2D.h>

// using Vector2fVector=std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>;
using ContainerType=LASERM::ContainerType;
using std::cerr; using std::endl;

std::unique_ptr<LASERM> laser_matcher;
ros::Publisher pub_2dpose;
Eigen::Isometry2f TB;
int num_msg=0;
int draw; //variable if you want to draw points for gnuplot, use as debugger
float sample_num=1; //to get a sample every sample_num 
// INPUT: laserscan, output pose2d and tf map->baselink 
void matcher_cb(const sensor_msgs::LaserScan &scan) {
  
  float angle_min = scan.angle_min;       
  float angle_max = scan.angle_max; 
  float angle_increment = scan.angle_increment;
  int size = std::ceil((angle_max-angle_min)/angle_increment/sample_num);
  
  // std::cerr << size << std::endl;
  // std::cerr << angle_min << std::endl;
  // std::cerr << angle_max << std::endl;
  // std::cerr << angle_increment << std::endl;

  cerr << "msg num" << num_msg << endl;
  if(num_msg==0) laser_matcher=std::unique_ptr<LASERM>(new LASERM(size,10,TB,draw));
  if(num_msg>1) {
    laser_matcher->updateOld();
    laser_matcher->resizeNiu(size);}
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
      continue;
    }
    idx++;
    float a = line*cos(angle);
    float b = line*sin(angle);
    laser_matcher->setSet(id,idx, Eigen::Vector2f(a,b));
  }
  
  num_msg++;
  if(num_msg==1) return;
  if(size!=laser_matcher->niu().size()){
    // cerr << "resizing" << endl;
    laser_matcher->resizeNiu(size);
  }
  cerr << "check seg" << endl;
  laser_matcher->run(1);
  cerr << "done check" << endl;
  laser_matcher->updateTB();

  auto tb = laser_matcher->TB();
  //update pose 2d
  geometry_msgs::Pose2D::Ptr pose_msg;
  pose_msg = boost::make_shared<geometry_msgs::Pose2D>();
  pose_msg->x = tb.translation()(0);
  pose_msg->y = tb.translation()(1);
  pose_msg->theta = Eigen::Rotation2Df(tb.rotation()).angle();
  pub_2dpose.publish(pose_msg);
  //update transform
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "base_link";
  transformStamped.transform.translation.x = pose_msg->x;
  transformStamped.transform.translation.y = pose_msg->y;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, pose_msg->theta);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  br.sendTransform(transformStamped);
  
}

int main(int argc, char **argv) {  
  if (argc<2) {
    cerr << "usage: " << argv[0] << " draw";
    return -1;
  }
  draw=atof(argv[1]);
  ros::init(argc, argv, "laser_matcher");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  pub_2dpose = n.advertise<geometry_msgs::Pose2D>("/pose2D", 1000);
  ros::Subscriber sub_lm = n.subscribe("/base_scan", 1000, matcher_cb);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;

  if (tfBuffer.canTransform("map", "base_link", ros::Time(0))) {
    transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
    // Eigen::Isometry3f test();
    tf2::Quaternion quat;
    tf2::convert(transformStamped.transform.rotation , quat);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // tf::Matrix3x3 mat(quat);
    // auto euler = q.eulerAngles(0, 1, 2);
    auto tr = transformStamped.transform.translation;
    TB.linear()=Rtheta(yaw);
    TB.translation()=Vector2f(tr.x, tr.y);
  }
  else{
    TB=Eigen::Isometry2f::Identity();
  }
  ros::spin();
}



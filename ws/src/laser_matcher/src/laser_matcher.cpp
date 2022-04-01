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
using std::cerr; using std::cout; using std::endl;

std::unique_ptr<LASERM> laser_matcher;
ros::Publisher pub_2dpose;
tf2_ros::Buffer tfBuffer;
int num_msg=0;
int draw; //parameter if you want to draw points for gnuplot, use as debugger
int tf_send; //parameter if you want to send tf computed and not only 2dpose
float sample_num=2; //to get a sample every sample_num, if you decrease it, increase number of _min_points_in_leaf to avoid segfault.

// Function to getTransform from->to
const Eigen::Isometry2f getTransform(const std::string& from, const std::string& to) {
  Eigen::Isometry2f TB = Eigen::Isometry2f::Identity();
  if(tfBuffer.canTransform(from, to, ros::Time(0))){
    geometry_msgs::TransformStamped transformStamped=tfBuffer.lookupTransform(from, to, ros::Time(0));
    tf2::Quaternion quat;
    tf2::convert(transformStamped.transform.rotation , quat);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // cerr <<"rpy "<< roll <<" "<< pitch <<" "<< yaw << endl;
    auto tr = transformStamped.transform.translation;
    TB.linear()=Rtheta(yaw);
    TB.translation()=Vector2f(tr.x, tr.y);
  }
  else{
	  cerr << "cannot transform correctly" << endl;
  }
  cerr << from << "->" << to << endl;
  cerr << TB.matrix() << endl;
  return TB;
  }

// INPUT: LaserScan, OUTPUT: pose2d and tf map->odom 
void matcher_cb(const sensor_msgs::LaserScan &scan) {
  
  float angle_min = scan.angle_min;       
  float angle_max = scan.angle_max; 
  float angle_increment = scan.angle_increment;
  int size = std::ceil((angle_max-angle_min)/angle_increment/sample_num);

  cerr << "msg num" << num_msg << endl;

  if(num_msg==0) {
    //to get the initial map->base_link transform to receive the right estimation from laser matcher
    Eigen::Isometry2f TMB=getTransform("map","base_link");
    Eigen::Isometry2f TBF=getTransform("base_link","base_laser_link");
    laser_matcher=std::unique_ptr<LASERM>(new LASERM(size,20,TMB*TBF,TBF,draw));
  }

  if(num_msg>1) {
    laser_matcher->updateOld();
    // laser_matcher->resizeNiu(size);
  }
  int id = num_msg!=0;
  
  float line;
  float angle=angle_min;
  int idx=0;
  for(int i=0; i<size; i+=1){
    line = scan.ranges[i*sample_num];
    angle += angle_increment*sample_num;
    // if (line > scan.range_max || line < scan.range_min){
    //   // cerr << "values < range_min or > range_max should be discarded" << endl;
    //   size--;
    //   continue;
    // }
    idx++;
    float a = line*cos(angle);
    float b = line*sin(angle);
    laser_matcher->setSet(id,idx, Eigen::Vector2f(a,b));
  }
  
  num_msg++;
  if(num_msg==1) return;
  if(size!=laser_matcher->niu().size()){
    // cerr << "resizing" << endl;
    // laser_matcher->resizeNiu(size);
  }
  //sometimes the creation of kdtree goes in segfault
  cerr << "check seg" << endl;
  laser_matcher->run(10);
  cerr << "done check" << endl;
  laser_matcher->updateTMF();

  auto tb = laser_matcher->TB();
  //update pose 2d
  cerr << "tb computed" << endl;
  cerr << tb.matrix() << endl;
  geometry_msgs::Pose2D::Ptr pose_msg;
  pose_msg = boost::make_shared<geometry_msgs::Pose2D>();
  pose_msg->x = tb.translation()(0);
  pose_msg->y = tb.translation()(1);
  pose_msg->theta = Eigen::Rotation2Df(tb.rotation()).angle();
  pub_2dpose.publish(pose_msg);
  //update transform, should use a parameter to print or not, a lot of warning if using a bag because of time http://wiki.ros.org/tf/Errors%20explained
  if(tf_send){
    auto tbo= getTransform("odom", "base_link").inverse();
    //tmo*tob=tmb => tmo=tmb*tob^-1
    auto tmo = tb*tbo;
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "/map";
    transformStamped.child_frame_id = "/odom";
    transformStamped.transform.translation.x = tmo.translation()(0);
    transformStamped.transform.translation.y = tmo.translation()(1);
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, Eigen::Rotation2Df(tmo.rotation()).angle());
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br.sendTransform(transformStamped);
  }
  //track origin of base_link frame 
  if(draw == 2){
    cout << "set size 1,1" << endl;
    cout <<"set xzeroaxis"<< endl;
    cout <<"set xtics axis"<< endl;
    cout <<"set xrange [-15:15]"<< endl;
    cout <<"set arrow 1 from -15,0 to -15,0"<< endl;
    cout <<"set arrow 2 from  15,0 to  15,0"<< endl;
    cout <<"set yzeroaxis"<< endl;
    cout <<"set ytics axis"<< endl;
    cout <<"set yrange [-10:10]"<< endl;
    cout <<"set arrow 3 from 0,-10,0 to 0,-10"<< endl;
    cout <<"set arrow 4 from 0,10,0  to 0,10"<< endl;
    cout <<"set border 0"<< endl;
    cout << "plot '-' w p ps 2" << endl;
    cout << tb.translation().transpose() << endl;
    cout << "e" << endl;
  }
}

int main(int argc, char **argv) {  
  ros::init(argc, argv, "laser_matcher");
  ros::NodeHandle n("~"); // private node handler to use parameters
  n.getParam("draw", draw);
  n.getParam("tfsend", tf_send);
  pub_2dpose = n.advertise<geometry_msgs::Pose2D>("/pose2D", 1000);
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Subscriber sub_lm = n.subscribe("/base_scan", 1000, matcher_cb);
  ros::spin();
  ros::shutdown();
}
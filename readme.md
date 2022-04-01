# Laser Matcher - RP Project 2021-2022
Following the hints of the professor I decided to use as base the solution for the exercise "rp_08_icp". The node subscribe to "/base_scan" and from it, starting from the known position of the robot, it will use LaserScan messages obtained by this subscription to correct the position odom->base_link given by the node that deals with at low encoder level. So the result is an updated map->odom tf and the estimated pose2D advertised in the ros environment. I managed to complete the project extending the professor ICP class to have an object laser_matcher of the class LASERM which stores, besides the standard variables to have the algorithms work, all the useful information to always have an updated map->base_link transformation that allows to easily compute the required tf.
## How to compile
The compilation process is made easy by the ros middleware. Arranging the workspace and package in the standard way the only commands needed to use this tool are:

```sh 
git clone https://github.com/DennisRotondi/rp_project.git
cd rp_project/ws
catkin init
catkin_make
source devel/setup.bash
```
## How to run
Ros made it easy even run it, once the setup.bash has been sourced is enough to use 
```sh
rosrun laser_matcher laser_matcher <parameters>
```
The rosparameter "tfsend" can be used to decide when publishing only the 2Dpose of the base_link in enough (tfsend==0) and when it isn't (tfsend!=0). 
## How to test
Although it is possible to generate the map from the bag using the SRRG suite or http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData (as I've done), and then set up the ros stack to have the robot in simulators like stage and rviz, I decided to use gnuplot so that is even easier to have feedback of what's going on, displaying either the evolution of the 2Dpose of the base_link wrt map or the walk of the matches between a scan and the other. 
The rosparameter "draw" can be used to choose between 2Dpose (draw==2), laser matches (draw==1) or nothing (draw!=1 && draw!=2).
Moreover, it is possible to check the result of what is being published on /pose2D using:
```sh
rostopic echo /pose2D
```
A possible testing run of the program that allows to display the draw of laser matches in gnuplot and meanwhile publish map->odom is:
```sh
rosrun laser_matcher laser_matcher _draw:=1 _tfsend:=1 | gnuplot
rosbag play rp_project/aula_41_2021-11-29-12-57-48.bag 
```
<img src="https://i.imgur.com/SaRSPSa.png" alt="gnuplot draw of a match between two LaserScan" width="500"/>

Possible improvements: to avoid clusters of points is possible to take one only when the norm is over a certain treshold; to improve the icp algorithm odom can be used as _X matrix

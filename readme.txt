to create the map from bag http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData

to draw the points
rosrun laser_matcher laser_matcher 1 0 | gnuplot
to draw the origin
rosrun laser_matcher laser_matcher 0 0 | gnuplot
to draw nothing
rosrun laser_matcher laser_matcher 0 0
to publish the map->base_link transform.
rosrun laser_matcher laser_matcher 0 1

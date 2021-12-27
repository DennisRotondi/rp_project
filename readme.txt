to create the map from bag http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData

to draw the points
rosrun laser_matcher laser_matcher _draw:=1 _tfsend:=1 | gnuplot
to draw the origin
rosrun laser_matcher laser_matcher _draw:=0 _tfsend:=0 | gnuplot
to draw nothing
rosrun laser_matcher laser_matcher _draw:=1 _tfsend:=0 | gnuplot
to publish the map->base_link transform.
rosrun laser_matcher laser_matcher _draw:=0 _tfsend:=1 | gnuplot

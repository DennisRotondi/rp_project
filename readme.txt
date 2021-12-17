to create the map from bag http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData

to draw the points
osrun laser_matcher laser_matcher 1 0 | gnuplot
to draw the origin
osrun laser_matcher laser_matcher 0 0 | gnuplot
to draw nothing
osrun laser_matcher laser_matcher 0 0
to publish the map->base_link transform.
osrun laser_matcher laser_matcher 0 1

#include <ros/ros.h>

#include <bob_toolbox/logging.h>
#include <bob_ros_implementations/ros_map_listener.h>
#include <bob_toolbox/world_point_shapes.h>
#include <bob_visualization/bob_toolbox_visualization.h>
#include <bob_stc/stc_planner.h>
#include <bob_coverage/area_tools.h>
#include <bob_coverage/discrete_area.h>
#include <bob_stc/grid_set.h>
#include <bob_stc/grid_generator.h>
#include <bob_visualization/visualization.h>
#include <bob_map_algorithms/obstacle_locator.h>
#include <bob_grid_map/lockable_map.h>
#include <bob_toolbox/strict_lock.h>
#include <iostream>

using namespace bob;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "grid_test");
	ros::NodeHandle nh;

	

	LOG_TEST("Test");
	
	// Markers for visualization
	

	// Used to obtain incoming map data 
	LockableMap lockableMap;
	ROSMapListener map(lockableMap);

	// We need to sleep to wait for the map data to come in (needs to be fixed later)
	ros::Duration(1.0).sleep();

	LOG_TEST("obstacle_detection lock");
	StrictLock lock(lockableMap.getLock());
	Costmap& costmap = lockableMap.getLockedResource(lock);
	ObstacleLocator locator(costmap);
	std::vector<MapLocationSet> obstacles = locator.getGroupedObstacles(WorldPoint(0, 0));

	for (std::vector<MapLocationSet>::iterator setItr = obstacles.begin(); setItr != obstacles.end(); ++setItr)
	{
		std::vector<WorldPoint> points = costmap.mapToWorld<std::vector<WorldPoint> >(*setItr);
		visualizer->visualize("obstacles", MarkerSquares(points));
		LOG_TEST("Press enter");
		std::cin.ignore();
	}
	LOG_TEST("Done");

	ros::spin();
	return 0;
}



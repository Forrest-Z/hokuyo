#include <ros/ros.h>

#include <bob_toolbox/logging.h>
#include <bob_ros_implementations/ros_map_listener.h>
#include <bob_grid_map/lockable_map.h>
#include <bob_coverage/visualization.h>
#include <bob_visualization/bob_toolbox_visualization.h>
#include <bob_toolbox/world_point_shapes.h>
#include <bob_stc/stc_planner.h>
#include <bob_coverage/area_tools.h>
#include <bob_coverage/discrete_area.h>
#include <bob_stc/grid_set.h>
#include <bob_stc/grid_generator.h>
#include <bob_visualization/visualization.h>
#include <iostream>

using namespace bob;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "grid_test");

	

	// Markers for visualization
	

	// Used to obtain incoming map data 
	LockableMap lockableMap;
	ROSMapListener map(lockableMap);

	// We need to sleep to wait for the map data to come in (needs to be fixed later)
	ros::Duration(1.0).sleep();

	LOG_TEST("stc_test lock");
	StrictLock lock(lockableMap.getLock());
	Costmap& costmap = lockableMap.getLockedResource(lock);

	// Define an arbitrary square in world co-ordinates
	WorldPoint squareCenter(3.0, 1.0);
	float squareWidth = 0.2;
	float rotation = 0.0;
	std::vector<WorldPoint> boundRegion = worldPointSquare(squareCenter, rotation, squareWidth);

	// Extract the areas of the costmap that we want to cover
	float bufferedRadius = 0.35;
	DiscreteArea area = extractThresholdedArea(costmap, boundRegion, bufferedRadius, 0.05);

	// Visualize that area
	visualizer->visualize("AreaToCover", visualization(area));

	GridGenerator gridGenerator(costmap.getObstacleMap());

	// Generate a STC grid solution for the area
	DiscreteArea coveredArea(0.05);
	GridSet solution = gridGenerator.generateSet(area, WorldPoint(0, 0), 0.0, coveredArea);
	//GridSet solution = gridGenerator.optimalSet(area, coveredArea);

	// The following doesnt work due to a bug:
	visualizer->visualize("CoveredArea", visualization(coveredArea));

	// This creates the plan for the robot (from several STC plans)
	//STCPlanner planner(costmap);
	//STCPlan plan = planner.createPlan(area);

	ros::spin();
	return 0;
}



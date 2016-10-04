#include <ros/ros.h>

#include <bob_ros_implementations/ros_map_listener.h>
#include <bob_toolbox/logging.h>

#include <bob_visualization/bob_toolbox_visualization.h>
#include <bob_toolbox/grid_wavefront_shell_iterator.h>
#include <bob_toolbox/grid_wavefront_full_iterator.h>
#include <bob_visualization/visualization.h>
#include <bob_grid_map/map_location_set.h>
#include <bob_grid_map/lockable_map.h>

#include <bob_map_algorithms/map_functor.h>

#include <bob_config/config.h>

using namespace bob;

int main(int argc, char** argv)
{

	ros::init(argc, argv, "wavefront_test");
	ros::NodeHandle nh;

	

	LOG_TEST("Test");

	// Markers for visualization
	


	// Used to obtain incoming map data 
	LockableMap lockableMap;
	ROSMapListener mapUpdater(lockableMap);

	// Wait for map data
	ros::Duration(15.0).sleep();	

	{
	// Lock the map from new data
	LOG_TEST("wavefront_test lock");
	StrictLock lock(lockableMap.getLock());
	const Costmap& costmap = lockableMap.getLockedResource(lock);

	MapLocationSet closedSet;
	MapLocationSet obstacleBoundary;
	MapLocation seed = costmap.worldToMap(WorldPoint(0, 1));

	NotMapFunctor::shared_ptr notFree(new NotMapFunctor(CellStateIs::shared_ptr(new CellStateIs(costmap, Free))));
	NotMapFunctor::shared_ptr nearObs(new NotMapFunctor(CellAwayFromObstacles::shared_ptr(new CellAwayFromObstacles(costmap, Config::ROBOT_RADIUS + 0.04))));
	OredMapFunctor::shared_ptr edgeCondition(new OredMapFunctor());
	edgeCondition->add(notFree);
	edgeCondition->add(nearObs);

	GridWavefrontFullIterator<MapLocationSet, MapFunctor> waveIterator(closedSet, seed, *edgeCondition);

	bool init = false;

	while (!waveIterator.done() && std::cin.ignore())
	{
		LOG_TEST("test");
		waveIterator.next();
		MapLocation currPoint = waveIterator.getCurrent();

		std::vector<WorldPoint> currWaveWP;
		currWaveWP.push_back(costmap.mapToWorld(currPoint));
		/*
		for(MapLocationSet::const_iterator itr = currWave.begin(); itr != currWave.end(); ++itr)
		{
			currWaveWP.push_back(costmap.mapToWorld(*itr));
		}
		*/
		visualizer->visualize("wave", MarkerSquares(currWaveWP));

		std::vector<WorldPoint> closedSetWP;
		for(MapLocationSet::const_iterator itr = closedSet.begin(); itr != closedSet.end(); ++itr)
		{
			closedSetWP.push_back(costmap.mapToWorld(*itr));
		}
		visualizer->visualize("close", MarkerSquares(closedSetWP), greenMarkerStyle());
	} 
	
	LOG_TEST("out of loop");
	}

	ros::spin();

	return 0;
}


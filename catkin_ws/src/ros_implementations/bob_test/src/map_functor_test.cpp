#include <ros/ros.h>

#include <bob_toolbox/logging.h>
#include <bob_ros_implementations/ros_map_listener.h>
#include <bob_coverage/area_tools.h>
#include <bob_grid_map/lockable_map.h>
#include <bob_toolbox/strict_lock.h>
#include <bob_map_algorithms/map_functor.h>

using namespace bob;

void pointState(const Costmap& costmap, WorldPoint point)
{
	MapLocation pointMap = costmap.worldToMap(point);
	FreeState state = costmap.getObstacleMap().pointFree(pointMap);
}

void mapFunctorStateTest(const Costmap& costmap, WorldPoint point)
{
	MapLocation pointMap = costmap.worldToMap(point);
	CellStateIs::shared_ptr free(new CellStateIs(costmap, Free));
	CellStateIs::shared_ptr obstacle(new CellStateIs(costmap, SensedObstacle));
	CellAwayFromObstacles::shared_ptr awayFromObs(new CellAwayFromObstacles(costmap, Config::ROBOT_RADIUS + 0.04));
	NotMapFunctor::shared_ptr nearObs(new NotMapFunctor(awayFromObs));
}

void mapFunctorCompositeStateTest(const Costmap& costmap, WorldPoint point)
{
	MapLocation pointMap = costmap.worldToMap(point);
	CellStateIs::shared_ptr free(new CellStateIs(costmap, Free));
	CellStateIs::shared_ptr obstacle(new CellStateIs(costmap, SensedObstacle));
	CellAwayFromObstacles::shared_ptr awayFromObs(new CellAwayFromObstacles(costmap, Config::ROBOT_RADIUS + 0.04));
	NotMapFunctor::shared_ptr nearObs(new NotMapFunctor(awayFromObs));

	// And
	AndedMapFunctor::shared_ptr freeAndAwayFromObs(new AndedMapFunctor());
	freeAndAwayFromObs->add(free);
	freeAndAwayFromObs->add(awayFromObs);	

	// Or
	OredMapFunctor::shared_ptr notFreeOrNearObs(new OredMapFunctor());
	notFreeOrNearObs->add(NotMapFunctor::shared_ptr(new NotMapFunctor(free)));
	notFreeOrNearObs->add(nearObs);
	
	// Not And
	NotMapFunctor::shared_ptr notFreeOrNearObs2(new NotMapFunctor(freeAndAwayFromObs));
	
	// Not Or
	NotMapFunctor::shared_ptr freeAndAwayFromObs2(new NotMapFunctor(notFreeOrNearObs));
	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_functor_test");
	ros::NodeHandle nh;

	// Used to obtain incoming map data 
	LockableMap lockableMap;
	ROSMapListener map(lockableMap);

	// We need to sleep to wait for the map data to come in (needs to be fixed later)
	ros::Duration(1.0).sleep();

	StrictLock lock(lockableMap.getLock());
	Costmap& costmap = lockableMap.getLockedResource(lock);

	WorldPoint freePoint(0, 0);
	WorldPoint obstaclePoint(0, -2);
	WorldPoint unknownPoint(-2, 0);

	LOG_TEST("state: 0: Free, 1: Obstacle, 2: Unknown");
	
	pointState(costmap, freePoint);
	mapFunctorStateTest(costmap, freePoint);
	mapFunctorCompositeStateTest(costmap, freePoint);

	pointState(costmap, obstaclePoint);
	mapFunctorStateTest(costmap, obstaclePoint);
	mapFunctorCompositeStateTest(costmap, obstaclePoint);
	
	
	pointState(costmap, unknownPoint);
	mapFunctorStateTest(costmap, unknownPoint);
	mapFunctorCompositeStateTest(costmap, unknownPoint);
	

//	ros::spin();
	return 0;
}



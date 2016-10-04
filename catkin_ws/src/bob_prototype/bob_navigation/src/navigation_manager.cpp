#include <bob_navigation/navigation_manager.h>

#include <bob_navigation/dijkstra_planner.h>

#include <bob_map_algorithms/map_functor.h>
#include <bob_lidar/util_functions.h>
#include <bob_map_algorithms/point_clear_for_navigation.h>

#include <bob_toolbox/geometry.h>
#include <bob_visualization/bob_toolbox_visualization.h>
#include <bob_toolbox/wave_expand_get_valid_point.h>
#include <bob_visualization/visualization.h>

#include <bob_toolbox/velocity2d.h>
#include <bob_toolbox/easy_print.h>
#include <bob_toolbox/logging.h>

#include <bob_control/robot_remote.h>
#include <bob_sensor/iproximity_sensor.h>

namespace bob
{

	NavigationManager::Result NavigationManager::navigationGoal(const WorldPoint& goal, IStopCondition::shared_ptr stopCondition) const
	{
		// Robot current position
		Pose2D pose = sensorHandle.getTransformHandle().getLocalizedPose();
	
		// Close enough that there is no need to do anything
		if (diagonalDistance<WorldPoint>(pose, goal) < 0.1)
			return NavigationManager::NavigationManager::ReachedGoal;

		std::vector<WorldPoint> plan;
		if (!generatePlan(pose, goal, plan))
		{
			return NavigationManager::PlanningFailed;
		}

		if (detectRecoveryNeeded())
		{
			LOG_NAVIGATION("recovery needed");
			executeRecovery();
		}

		IPathExecutor::Result result = pathExecutor.run(plan, stopCondition);
		ControlResult simpleResult = result.lowLevelResult;	

		if (simpleResult == ObstacleSafety || simpleResult == BumperHit)
		{
			// Call this function recursively if we are interrupted due to bump or obstacle
			Result newResult = navigationGoal(plan.back(), stopCondition);
			if (newResult == PlanningFailed)
			{
				return Aborted;
			}
			else
			{
				return newResult;
			}
		}
		
		if (stopCondition->wasSatisfied())
		{
			return NavigationManager::StopCondition;
		}
		else
		{
			return NavigationManager::ReachedGoal;
		}
	}

	bool NavigationManager::generatePlan(Pose2D start, WorldPoint goal, std::vector<WorldPoint>& plan) const
	{
		// Lock map
		StrictLock lock(lockableMap.getLock());
		const Costmap& costmap = lockableMap.getLockedResource(lock);

		PlanningTask task = producePlanningTask(start, goal, costmap);

		bool planningSuccess = makePlan(task.start, task.goal, plan, costmap);
		if(!planningSuccess)
		{
			LOG_ERROR("Failed to make plan");
			return false;
		}

		return true;
	}

	NavigationManager::PlanningTask NavigationManager::producePlanningTask(Pose2D start, WorldPoint goal, const Costmap& costmap) const
	{
		PlanningTask toReturn;
		toReturn.start = (WorldPoint)start;
		toReturn.goal = goal;

		Velocity2D currentVelocity = sensorHandle.getOdometryHandle().getRobotVel();

		// Check if the goal is valid
		if (!pointClearForNavigation(costmap, goal))
		{
			// Search for valid goal around the invalid goal
			toReturn.goal = expandGetClearPoint(costmap, toReturn.goal);
		}

		// If the robot is moving, choose a point in front of current 
		// position as the actual start point.
		if (fabs(currentVelocity.x) > 0.02)
		{
			toReturn.start = shiftedPoint(start, currentVelocity.x * 1.0);
		}

		// Check if current position is free and away from obstacle, 
		// if not expand and find the closest clear point.
		if (!pointClearForNavigation(costmap, toReturn.start))
		{
			toReturn.start = expandGetClearPoint(costmap, toReturn.start);
		}

		return toReturn;
	}

	bool NavigationManager::detectRecoveryNeeded() const
	{
		float frontDistance = sensorHandle.getProximitySensor().distanceFromRobot(0.0);
		return (frontDistance < 0.05) && canBackUp();
	}

	bool NavigationManager::canBackUp() const
	{
		float backDistance = sensorHandle.getProximitySensor().distanceFromRobot(M_PI);
		return (backDistance > 0.03);	
	}

	void NavigationManager::executeRecovery() const
	{
		RobotRemote robotRemote(simpleCommander, sensorHandle);	
		robotRemote.backUp(0.03);
	}


	bool NavigationManager::pointClearForNavigation(const Costmap& costmap, const WorldPoint& point) const
	{
		MapLocation pointMapLocation = costmap.worldToMap(point);


		CellStateIs::shared_ptr free(new CellStateIs(costmap, Free));
		CellAwayFromObstacles::shared_ptr awayFromObs(new CellAwayFromObstacles(costmap, Config::ROBOT_RADIUS + 0.04));
		AndedMapFunctor::shared_ptr freeAndAwayFromObs(new AndedMapFunctor());
		freeAndAwayFromObs->add(free);
		freeAndAwayFromObs->add(awayFromObs);

		return (*freeAndAwayFromObs)(pointMapLocation);
	}

	WorldPoint NavigationManager::expandGetClearPoint(const Costmap& costmap, const WorldPoint& seed) const
	{
		if(pointClearForNavigation(costmap, seed))
		{
			return seed;
		}

		MapLocation seedMapLocation = costmap.worldToMap(seed);

		CellStateIs::shared_ptr free(new CellStateIs(costmap, Free));
		CellAwayFromObstacles::shared_ptr awayFromObs(new CellAwayFromObstacles(costmap, Config::ROBOT_RADIUS + 0.04));
		AndedMapFunctor::shared_ptr freeAndAwayFromObs(new AndedMapFunctor());
		freeAndAwayFromObs->add(free);
		freeAndAwayFromObs->add(awayFromObs);

		MapLocation validPoint;
		waveExpandGetValidPoint(costmap, seedMapLocation, *freeAndAwayFromObs, 81, validPoint);

		return costmap.mapToWorld(validPoint);
	}

	bool NavigationManager::makePlan(const WorldPoint& start, const WorldPoint& goal, std::vector<WorldPoint>& result, const Costmap& costmap) const
	{
		std::vector<WorldPoint> plan;
		std::vector<WorldPoint> optimizedPlan;

		visualizer->visualize("planner_goal", goal, greenMarkerStyle());

		// Lock map to prevent modification while using
		DijkstraPlanner planner(costmap);

		// If the planner fails or returns a zero length plan, planning failed
		if(planner.makePlan(start, goal, plan))
		{
			// Simplize plan with line segment
			optimizedPlan = optimizePlan(costmap, plan);
			if(optimizedPlan.size() >= 2)
			{
				visualizer->visualize("plan", MarkerLine(optimizedPlan), blueMarkerStyle());
				result = optimizedPlan;
				return true;
			}
		}
		return false;	
	}

	WorldPoint NavigationManager::shiftedPoint(const Pose2D& robotPose, float checkDistance) const
	{
		WorldPoint actualStart = robotPose + checkDistance * unitVector(robotPose.theta);
		return actualStart;
	}

}

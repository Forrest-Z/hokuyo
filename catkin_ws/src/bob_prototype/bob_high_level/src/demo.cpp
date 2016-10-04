#include <bob_high_level/demo.h>

#include <bob_grid_map/toolbox.h>
#include <bob_core/alternating_exploration_coverage.h>
#include <bob_visualization/bob_toolbox_visualization.h>
#include <bob_coverage/visualization.h>
#include <bob_visualization/visualization.h>

//#include <bob_frontier_exploration/frontier_explorer.h>
#include <bob_frontier_exploration/frontier_chooser.h>
#include <bob_frontier_exploration/frontier_condition.h>


#include <bob_toolbox/world_point_shapes.h>
#include <bob_toolbox/easy_print.h>
#include <bob_system/periodic_threader.h>
#include <bob_toolbox/logging.h>

#include <bob_control/straight_path_executor.h>
#include <bob_control/straight_driver.h>
#include <bob_control/robot_remote.h>
#include <bob_wall_following/wall_follower.h>
#include <bob_wall_following/wall_follow_back_condition.h>
#include <bob_wall_following/wall_follow_overlap_condition.h>

#include <bob_stc/stc_planner.h>
#include <bob_stc/stc_plan_executor.h>
#include <bob_stc/visualization.h>
#include <bob_stc/stc.h>
#include <bob_coverage/area_tools.h>

#include <bob_boustrophedon/boustrophedon.h>

namespace bob
{

	Demo::Demo(ISensorHandle& sensorHandle, LockableMap& lockableMap, IVelocityPublisher& velocityPublisher) :
		velocityPublisher(velocityPublisher),
		sensorHandle(sensorHandle),
		lockableMap(lockableMap),
		controlHandle(sensorHandle, lockableMap, velocityPublisher),	
		areaProcessor(lockableMap, sensorHandle.getTransformHandle())
	{}

	void Demo::operator()()
	{
		waitForMapAvailable(lockableMap);
		
		areaProcessor.start();

		alternateExplorationAndCoverage();

		/*
		LOG_HIGH_LEVEL("Beginning exploration");
		exploreEverything();
		LOG_HIGH_LEVEL("Exploration done.");

		LOG_HIGH_LEVEL("Beginning STC");
		stcCoverage();
		//	boustrophedonCoverage();
		LOG_HIGH_LEVEL("STC Coverage done.");
		*/

		LOG_HIGH_LEVEL("Wall following obstacles");
		wallFollowObstacles();

		LOG_HIGH_LEVEL("Back to start");
		backToStart();
		

	}	

	void Demo::alternateExplorationAndCoverage()
	{
		Boustrophedon boustrophedonAlgorithm(controlHandle, sensorHandle);
		AlternatingExplorationCoverage algorithm(lockableMap, sensorHandle, controlHandle, areaProcessor, boustrophedonAlgorithm);
		algorithm();
	}

	void Demo::stcCoverage()
	{
		/*
		STCPlan stcPlan;
		{
			StrictLock lock(lockableMap.getLock());
			const Costmap& costmap = lockableMap.getLockedResource(lock);
			STCPlanner stcPlanner(costmap.getObstacleMap());

			// Define an arbitrary square in world co-ordinates
			WorldPoint squareCenter = sensorHandle.getTransformHandle().getLocalizedPose();
			float squareWidth = 6;
			float rotation = 0.0;
			std::vector<WorldPoint> boundRegion = worldPointSquare(squareCenter, rotation, squareWidth);

			// Extract the areas of the costmap that we want to cover
			float bufferedRadius = 0.35;
			DiscreteArea planningArea = extractThresholdedArea(costmap, boundRegion, bufferedRadius, 0.05);

			visualizer->visualize("STC_Area", visualization(planningArea));

			stcPlan = stcPlanner.createPlan(planningArea);
			visualizer->visualize("STC_plan", visualization(visualization(stcPlan)));
			
		}

		CurvedPathExecutor curvedPathExecutor(controlHandle.simpleCommander, sensorHandle.getTransformHandle());
		STCPlanExecutor stcPlanExecutor(curvedPathExecutor, controlHandle.navigationManager, lockableMap);
		stcPlanExecutor.execute(stcPlan);
		*/
	}


	void Demo::boustrophedonCoverage()
	{
		//	boustrophedonExecutor.execute();
	}

	void Demo::wallFollowObstacles()
	{
		// In case robot current position is not clear (too close to obstacle)
		controlHandle.navigationManager.navigationGoal(sensorHandle.getTransformHandle().getLocalizedPose());
		WallChooser wallChooser(lockableMap, sensorHandle.getTransformHandle());
		while(!wallChooser.done())
		{
			WorldPoint wallPoint = wallChooser.nextWallPoint();
			controlHandle.navigationManager.navigationGoal(wallPoint);
			WallFollower wallFollower(controlHandle.simpleCommander, sensorHandle, RightSide);
			//WallFollowBackCondition::shared_ptr condition(new WallFollowBackCondition(sensorHandle.getTransformHandle().getLocalizedPose(), 0.2, 0.15));
			WallFollowOverlapCondition::shared_ptr condition(new WallFollowOverlapCondition());
			wallFollower.run(condition);
			visualizer->visualize("total_area", visualization(areaProcessor.getCoverableSpace()));
		}
	}
	
	void Demo::backToStart()
	{
		controlHandle.navigationManager.navigationGoal(WorldPoint(-0.1, 0));
		RobotRemote robotRemote(controlHandle.simpleCommander, sensorHandle);
		robotRemote.headingRotation(0);		
		robotRemote.goForward(0.1);
	}
}

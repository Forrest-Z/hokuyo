#ifndef _BOB_NAVIGATION_NAVIGATION_MANAGER_H_
#define _BOB_NAVIGATION_NAVIGATION_MANAGER_H_

#include <bob_grid_map/lockable_map.h>
#include <bob_sensor/itransform_handle.h>

#include <bob_navigation/path_splitter.h>

#include <bob_control/simple_commander.h>
#include <bob_control/curved_path_executor.h>
#include <bob_control/conditions/istop_condition.h>
#include <bob_control/conditions/null_condition.h>

#include <bob_toolbox/world_point.h>
#include <bob_sensor/iodometry_handle.h>

#include <bob_toolbox/pose2d.h>
#include <bob_control/conditions/istop_condition.h>
#include <bob_control/conditions/null_condition.h>
#include <bob_sensor/isensor_handle.h>

namespace bob
{

	//! \brief Produces behavior which drives the robot from one point in the map to the other.
	//! Once a goal is given, the NavigationManager object will attemp to reach it by planning a path
	//! to the goal and driving the robot along that path until the goal is reached.
	//! If an hidden obstacle is encountered along the way, the robot will re-plan with the new
	//! map containing the hidden obstacle.
	class NavigationManager
	{

		public:

			//! The result of a navigation task
			enum Result
			{

				//! A plan to the goal could not be found. It is unreachable
				PlanningFailed,

				//! The navigation task stopped early due to the option additional stop condition provided to the algorithm
				StopCondition,
		
				//! The goal was successfully reached
				ReachedGoal,	

				//! The robot started towards the goal, but gave up because it was found to be unreachable, likely due to
				//! hidden obstacles
				Aborted	

			};

			//! \brief Create a NavigationManager instance
			NavigationManager(CurvedPathExecutor& pathExecutor, const LockableMap& lockableMap, SimpleCommander& simpleCommander, const ISensorHandle& sensorHandle):
				pathExecutor(pathExecutor),
				lockableMap(lockableMap),
				simpleCommander(simpleCommander),
				sensorHandle(sensorHandle)
		{}

			//! \brief Execute a navigation task
			//! \param goal The goal point for the robot to travel to
			//! \param stopCondition An optional condition can be provided which will stop the robot early, before the goal is reached
			//! \return The result of the task
			Result navigationGoal(const WorldPoint& goal, IStopCondition::shared_ptr stopCondition = NullCon) const;


		private:

			struct PlanningTask
			{

				WorldPoint start;
				WorldPoint goal;

			};

			CurvedPathExecutor& pathExecutor;

			const LockableMap& lockableMap;

			SimpleCommander& simpleCommander;

			const ISensorHandle& sensorHandle;

			PlanningTask producePlanningTask(Pose2D start, WorldPoint goal, const Costmap& costmap) const;

			bool generatePlan(Pose2D start, WorldPoint goal, std::vector<WorldPoint>& plan) const;

			bool pointClearForNavigation(const Costmap& costmap, const WorldPoint& point) const;

			WorldPoint expandGetClearPoint(const Costmap& costmap, const WorldPoint& seed) const;
	
			bool makePlan(const WorldPoint& start, const WorldPoint& goal, std::vector<WorldPoint>& result, const Costmap& costmap) const;

			WorldPoint shiftedPoint(const Pose2D& robotPose, float checkDistance) const;

			bool detectRecoveryNeeded() const;

			void executeRecovery() const;

			bool canBackUp() const;

	};

}

#endif

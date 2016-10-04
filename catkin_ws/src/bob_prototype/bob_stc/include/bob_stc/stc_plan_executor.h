#ifndef _BOB_STC_STC_PLAN_EXECUTOR_H_
#define _BOB_STC_STC_PLAN_EXECUTOR_H_

#include <bob_stc/stc_plan.h>

#include <bob_navigation/navigation_manager.h>
#include <bob_control/ipath_executor.h>

#include <bob_toolbox/line_segment.h>
#include <bob_toolbox/strict_lock.h>

#include <bob_grid_map/lockable_map.h>

namespace bob
{

	class STCPlanExecutor
	{

		public:

			STCPlanExecutor(IPathExecutor& pathExecutor, NavigationManager& navigationManager, const LockableMap& lockableMap) : 
				pathExecutor(pathExecutor),
				navigationManager(navigationManager),
				lockableMap(lockableMap) {}

			void execute(STCPlan plan);

		private:

			typedef std::vector<IPathExecutor::Path> CombinedPaths;

			CombinedPaths processPlan(const STCPlan& plan);

			//! Determines if the robot can pass a line segment without hitting walls
			bool lineSegmentSafe(LineSegment line, const IObstacleDistanceMap& inflatedMap);

	
			IPathExecutor& pathExecutor;
			NavigationManager& navigationManager;
			const LockableMap& lockableMap;

			

	};

}

#endif

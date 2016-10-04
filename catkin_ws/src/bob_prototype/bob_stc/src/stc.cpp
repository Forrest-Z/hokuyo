#include <bob_stc/stc.h>

#include <bob_stc/stc_plan_executor.h>
#include <bob_control/curved_path_executor.h>
#include <bob_toolbox/strict_lock.h>
#include <bob_stc/visualization.h>
#include <bob_coverage/visualization.h>
#include <bob_stc/stc_planner.h>
#include <bob_visualization/bob_toolbox_visualization.h>
#include <bob_sensor/isensor_handle.h>
#include <bob_control_handle/control_handle.h>
#include <bob_grid_map/lockable_map.h>
#include <bob_visualization/visualization.h>

namespace bob
{

	void STC::coverArea(const DiscreteArea& area)
	{
		STCPlan stcPlan;
		{
			StrictLock lock(lockableMap.getLock());
			const Costmap& costmap = lockableMap.getLockedResource(lock);

			visualizer->visualize("STC_Area", visualization(area));

			STCPlanner stcPlanner(costmap.getObstacleMap());
			stcPlan = stcPlanner.createPlan(area);
			visualizer->visualize("STC_plan", visualization(stcPlan));
		}

		CurvedPathExecutor curvedPathExecutor(controlHandle.simpleCommander, sensorHandle.getTransformHandle());
		STCPlanExecutor stcPlanExecutor(controlHandle.pathExecutor, controlHandle.navigationManager, lockableMap);
		stcPlanExecutor.execute(stcPlan);
	}

}


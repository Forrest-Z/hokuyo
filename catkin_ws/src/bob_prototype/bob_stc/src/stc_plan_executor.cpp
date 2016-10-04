#include <bob_stc/stc_plan_executor.h>

#include <bob_config/config.h>

#include <bob_grid_map/concrete_inflated_map.h>

#include <bob_toolbox/raytrace.h>

#include <boost/utility.hpp>

namespace bob
{

	void STCPlanExecutor::execute(STCPlan plan)
	{
		CombinedPaths combined = processPlan(plan);
		for (CombinedPaths::const_iterator pathItr = combined.begin(); pathItr != combined.end(); ++pathItr)
		{
			if (pathItr->size() <= 1)
			{
				// Skip erroneous paths that have length less than 1
				continue;
			}
			
			navigationManager.navigationGoal(pathItr->front());
			pathExecutor.run(*pathItr);
		}
	}

	STCPlanExecutor::CombinedPaths STCPlanExecutor::processPlan(const STCPlan& plan)
	{
		// Lock the map
		StrictLock lock(lockableMap.getLock());
		const Costmap& costmap = lockableMap.getLockedResource(lock);

		CombinedPaths result;
		IPathExecutor::Path tempPath;
		for (STCPlan::const_iterator pathItr = plan.begin(); pathItr != plan.end(); ++pathItr)
		{
			tempPath.insert(tempPath.end(), pathItr->begin(), pathItr->end());	
			if (pathItr == boost::prior(plan.end()))
			{
				// Last path, so just insert it into result
				result.push_back(tempPath);
			}
			else
			{
				// Line joining successive paths
				LineSegment connectingLine(pathItr->back(), boost::next(pathItr)->front());
				if (!lineSegmentSafe(connectingLine, costmap.getObstacleDistanceMap()))
				{
					// We must have a split here where we use navigation, wall following, etc.
					// because there is an obstacle, so we cannot move straight
					result.push_back(tempPath);
					tempPath.clear();	
				}
				// No else statement needed. The line segment will be connected when adding the	
				// next path in the next for loop iteration.
			}
		}
		return result;	
	}
	
	bool STCPlanExecutor::lineSegmentSafe(LineSegment line, const IObstacleDistanceMap& inflatedMap)
	{
		MapLocation firstPoint = inflatedMap.worldToMap(line.first);
		MapLocation secondPoint = inflatedMap.worldToMap(line.second);
		std::vector<MapLocation> relevantPoints = raytraceLine(firstPoint, secondPoint);
		for (std::vector<MapLocation>::iterator mapLocationItr = relevantPoints.begin(); mapLocationItr != relevantPoints.end(); ++mapLocationItr)
		{
			float safetyBuffer = 0.1;
			float minAllowedDistance = Config::ROBOT_RADIUS + safetyBuffer;

			if (inflatedMap.obstacleDistance(*mapLocationItr) < minAllowedDistance)
			{
				return false;
			}
		}
		return true;
	}
	
}

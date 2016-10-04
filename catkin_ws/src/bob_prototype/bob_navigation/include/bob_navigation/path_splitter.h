#ifndef _BOB_NAVIGATION_PATH_SPLITTER_H_
#define _BOB_NAVIGATION_PATH_SPLITTER_H_

#include <bob_grid_map/costmap.h>
#include <bob_toolbox/world_point.h>
#include <bob_toolbox/grid_algorithms.h>
#include <vector>
#include <cmath>
#include <limits>


namespace bob
{

	//! Replace path with effective line segments.
	std::vector<WorldPoint> optimizePlan(const Costmap& costmap, const std::vector<WorldPoint>& plan);

	float minObstacleDistance(const Costmap& costmap, const std::vector<WorldPoint>& points);

	std::vector<WorldPoint> optimizePlanWithObstacleThreshold(const Costmap& costmap, const std::vector<WorldPoint>& plan, float threshold);

	std::vector<WorldPoint>::const_iterator optimizeAheadOfPoint(const Costmap& costmap, const std::vector<WorldPoint>& plan, float threshold, std::vector<WorldPoint>& optimizedPlan, std::vector<WorldPoint>::const_iterator fromPoint);

};

#endif

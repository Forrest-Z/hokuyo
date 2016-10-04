#include <bob_navigation/path_splitter.h>

#include <bob_toolbox/raytrace.h>
#include <bob_map_algorithms/map_functor.h>
#include <bob_map_algorithms/helper_functions.h>

#include <assert.h>

namespace bob
{
	std::vector<WorldPoint> optimizePlan(const Costmap& costmap, const std::vector<WorldPoint>& plan)
	{
		// If plan length smaller than 3, there is no need to optimize
		if(plan.size() < 3 && plan.empty())
			return std::vector<WorldPoint>(plan);

		// Calculate the minimum obstacle distance among the points in the plan
		float minAllowedDistance = minObstacleDistance(costmap, plan);
		
		return optimizePlanWithObstacleThreshold(costmap, plan, minAllowedDistance);
	}

	std::vector<WorldPoint> optimizePlanWithObstacleThreshold(const Costmap& costmap, const std::vector<WorldPoint>& plan, float threshold) 
	{
		std::vector<WorldPoint> optimizedPlan;
		std::vector<WorldPoint>::const_iterator fromPoint = plan.begin();
		optimizedPlan.push_back(*fromPoint);
		while(fromPoint != plan.end())
		{
			std::vector<WorldPoint>::const_iterator newFromPoint = optimizeAheadOfPoint(costmap, plan, threshold, optimizedPlan, fromPoint);	
			assert(newFromPoint != fromPoint);
			fromPoint = newFromPoint;
		}

		// Add last point in new plan.
		optimizedPlan.push_back(plan.back());

		return optimizedPlan;
	}

	std::vector<WorldPoint>::const_iterator optimizeAheadOfPoint(const Costmap& costmap, const std::vector<WorldPoint>& plan, float threshold, std::vector<WorldPoint>& optimizedPlan, std::vector<WorldPoint>::const_iterator fromPoint)
	{
			// Convert world point to map point
			MapLocation from = costmap.getObstacleDistanceMap().worldToMap(*fromPoint);
			
			// Flag for if raytrace hit obstacle
			bool collision = false;
			std::vector<WorldPoint>::const_iterator toPoint = fromPoint + 1;
			for(; toPoint != plan.end(); ++toPoint)
			{	
				MapLocation to = costmap.getObstacleDistanceMap().worldToMap(*toPoint);

				// Raytrace following points, check collision
				std::vector<MapLocation> raytracedCells = raytraceLine(from, to);
				
				CellStateIs::shared_ptr free(new CellStateIs(costmap, Free));		
				CellAwayFromObstacles::shared_ptr awayFromObs(new CellAwayFromObstacles(costmap, threshold));
				//AndedMapFunctor::shared_ptr freeAndAwayFromObs(new AndedMapFunctor());
				//freeAndAwayFromObs->add(free);
				//freeAndAwayFromObs->add(awayFromObs);
			
				collision = !allPointsSatisfy(raytracedCells, *awayFromObs);
		 
				if(collision)
				{
					// If raytrace hit high cost cell, add previous point into new plan, then break;
					optimizedPlan.push_back(*(toPoint - 1));
					break;
				}
			}
			
			// If raytrace hit high cost cell, next loop start from j-1, otherwise start from j.
			fromPoint = collision ? (toPoint - 1) : toPoint;	
			return fromPoint;
	}

	float minObstacleDistance(const Costmap& costmap, const std::vector<WorldPoint>& points)
	{
		// Find the max cost in the plan
		float minPlanDistance = std::numeric_limits<int>::max();
		for(std::vector<WorldPoint>::const_iterator itr = points.begin(); itr != points.end(); ++itr)
		{
			MapLocation location = costmap.worldToMap(*itr);
			float distanceFromObstacles = costmap.getObstacleDistanceMap().obstacleDistance(location);

			if(distanceFromObstacles < minPlanDistance)
				minPlanDistance = distanceFromObstacles;
		}
		return minPlanDistance;
	}
};

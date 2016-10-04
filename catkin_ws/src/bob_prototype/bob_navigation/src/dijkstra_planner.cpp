#include <bob_navigation/dijkstra_planner.h>

#include <bob_toolbox/eight_direction.h>
#include <bob_grid_map/costmap.h>

#include <bob_config/config.h>

#include <algorithm>

namespace bob
{

	const float DijkstraPlanner::crossFactor = sqrt(2);	

	bool DijkstraPlanner::makePlan(WorldPoint start, WorldPoint goal, std::vector<WorldPoint>& plan)
	{
		reset();

		MapLocation mapStart = potential.worldToMap(start);
		MapLocation mapGoal = potential.worldToMap(goal);

		if (!computePotential(mapStart, mapGoal))
			return false;

		plan = computePlanFromPotential(mapStart, mapGoal);

		return true;
	}

	void DijkstraPlanner::reset()
	{
		potential.resizeClear(map.getObstacleMap().getMapMetadata());
		closedSet.clear();
		openQueue = std::priority_queue<PotentialPoint>();
	}

	bool DijkstraPlanner::computePotential(MapLocation start, MapLocation goal)
	{
		// Add the starting point to the priority queue
		openQueue.push(PotentialPoint(start, 0));		

		// Setup the potential of the start point.
		// Otherwise the potential will never be set and
		// the planner wont work if start == goal
		DijkstraPoint startPotential(start, 0);
		potential[start] = startPotential;	

		while (!openQueue.empty())
		{
			// Any MapLocation in the queue will already have a back pointer path to start
			PotentialPoint openPoint = openQueue.top();
			openQueue.pop();

			// We have found the goal, so computing the potential is done
			if (openPoint.location == goal)
				return true;

			// We might have duplicates in the queue, so ignore points that come up twice
			if (closedSet.count(openPoint.location) == 1)
				continue;

			// Don't expand points if they travel too far into unknown space
			if (distanceTravelledInUnknown(openPoint.location) >= 5)
				continue;

			// Check all points horizontally
			expandPointsInDirections(nondiagonalEightDirections(), openPoint, 1);

			// Check all points diagonally (note the sqrt(2) because we are travelling diagonally)
			expandPointsInDirections(diagonalEightDirections(), openPoint, crossFactor);

			// Close the point so it is not re-opened
			closedSet.insert(openPoint.location);			
		}
		return false;
	}

	void DijkstraPlanner::expandPointsInDirections(std::vector<EightDirection> directionsToCheck, PotentialPoint source, float factor)
	{
		for (std::vector<EightDirection>::iterator directionItr = directionsToCheck.begin();
				directionItr != directionsToCheck.end();
				++directionItr)
		{
			MapLocation newPoint = cellInDirection(source.location, *directionItr);

			// Calculate the potential of moving between two points
			float newPotential = source.potential + factor * (calculatePotential(source.location) + calculatePotential(newPoint)) / 2;		

			// If the newPotential is less, than accept it
			if (newPotential < potential.getValue(newPoint).potential)
			{
				DijkstraPoint potentialValue(source.location, newPotential);

				// Note, only add point to the queue if the point existed in the map
				if (potential.isInside(newPoint))
				{
					potential[newPoint] = potentialValue;
					openQueue.push(PotentialPoint(newPoint, newPotential));
				}
			}
		}
	}

	float DijkstraPlanner::calculatePotential(MapLocation point)
	{
		float cost;
		if (map.getObstacleMap().pointFree(point) == SensedObstacle || 
			map.getObstacleMap().pointFree(point) == HiddenObstacle)
		{
			return std::numeric_limits<float>::infinity();
		}
		else
		{
			float obstacleDistance = map.getObstacleDistanceMap().obstacleDistance(point);
			float radius = Config::ROBOT_RADIUS + 0.02;
			float clearance = obstacleDistance - radius;
			if (clearance < 0)
			{
				return std::numeric_limits<float>::infinity();
			}
			else
			{
				cost = 50 + 0.8 * (int)((253 - 1) * exp(-10 * (clearance)));
			}
		}

		if (map.getObstacleMap().pointFree(point) == Unknown)
		{
			// penalty for unknown space
			cost += 50;
		}
		return cost;
	}

	int DijkstraPlanner::distanceTravelledInUnknown(MapLocation point)
	{
		int distance = 0;
		while (map.getObstacleMap().pointFree(point) == Unknown)
		{
			distance++;
			point = potential.getValue(point).previous;
		}
		return distance;
	}

	std::vector<WorldPoint> DijkstraPlanner::computePlanFromPotential(MapLocation start, MapLocation goal)
	{
		std::vector<WorldPoint> plan;
		plan.push_back(potential.mapToWorld(goal));

		// Keep going along the pointers to previous until we reach start
		DijkstraPoint openPoint = potential.getValue(goal);
		while (openPoint.previous != start)
		{
			plan.push_back(potential.mapToWorld(openPoint.previous)); 
			openPoint = potential.getValue(openPoint.previous);
		}

		plan.push_back(potential.mapToWorld(start));

		// Reverse that plan, since we went backwards
		std::reverse(plan.begin(), plan.end());
		return plan;
	}

}



#include <bob_coverage/area_tools.h>

#include <algorithm>
#include <vector>
#include <utility>
#include <limits>
#include <cmath>

#include <bob_toolbox/world_point.h>

#include <bob_toolbox/grid_algorithms.h>
#include <bob_toolbox/geometry.h>

namespace bob
{

	float smallestDistanceTo(const DiscreteArea& area, WorldPoint point)
	{
		MapLocation pointLocation = area.worldToMap(point);
		float smallestSoFar = std::numeric_limits<float>::max();
		for (DiscreteArea::const_iterator areaPointItr = area.begin(); areaPointItr != area.end(); ++areaPointItr)
		{
			float nextDistance = diagonalDistance<MapLocation>(*areaPointItr, pointLocation);
			smallestSoFar = std::min(smallestSoFar, nextDistance);
		}
		return smallestSoFar;
	}

	// TODO: This function should probably accept a functor instead of a minDistance, to make it more flexible
	DiscreteArea extractThresholdedArea(const Costmap& map, const std::vector<WorldPoint>& convexPolygon, float minDistance, float resolution)
	{
		DiscreteArea mask(resolution);

		auto discretePolygon = mask.worldToMap< std::vector<MapLocation> >(convexPolygon);

		// Get all map points that are inside the polygon
		auto allPoints = convexFillCells(discretePolygon);

		// Add all the points within the threshold into the returned vector
		for (auto pointItr = allPoints.begin(); pointItr != allPoints.end(); ++pointItr)
		{
			// We must convert like this because it's possible that the map has a different
			// resolution than the DiscreteArea.
			// TODO: if statement to Only make the conversion of the resolutions are different	
			WorldPoint point = mask.mapToWorld(*pointItr);
			MapLocation location = map.worldToMap(point);

			// Checking the condition
			bool satisfiesDistance = (map.getObstacleDistanceMap().obstacleDistance(location) >= minDistance);
			bool isFree = (map.getObstacleMap().pointFree(location) == Free);
			if (satisfiesDistance && isFree)
				mask.insert(*pointItr);
		}
		return mask;
	}

	WorldRectangle minAreaBoundingBox(const DiscreteArea& area) 
	{
		float angleIncrement = M_PI / 90;
		float angle = 0;
		float minArea = std::numeric_limits<float>::max();
		float minAreaAngle;
		while(angle < M_PI / 2)
		{
			WorldRectangle box = boundingBox(area, angle);
			if(box.getArea() < minArea)
			{
				minArea = box.getArea();
				minAreaAngle = angle;
			}
			angle += angleIncrement;
		}
		
		return boundingBox(area, minAreaAngle);
	}

	WorldRectangle boundingBox(const DiscreteArea& area, float angle) 
	{
		float minX = std::numeric_limits<float>::max();
		float maxX = -std::numeric_limits<float>::max();
		float minY = std::numeric_limits<float>::max();
		float maxY = -std::numeric_limits<float>::max();

		for(auto itr = area.begin(); itr != area.end(); ++itr)
		{
			WorldPoint worldPoint = area.mapToWorld(*itr);
			
			WorldPoint pt =	rotate(worldPoint, -angle);

			if(pt.x < minX)
			{
				minX = pt.x;
			}			

			if(pt.x > maxX)
			{
				maxX = pt.x;
			}

			if(pt.y < minY)
			{
				minY = pt.y;
			}

			if(pt.y > maxY)
			{
				maxY = pt.y;
			}
		}	
		
		WorldPoint origin = rotate(WorldPoint(minX, minY), angle);

		WorldRectangle result;
		result.bounds.width = (maxX - minX);
		result.bounds.height = (maxY - minY);
		result.origin = origin - WorldPoint(0, 0);
		result.angle = angle;

		return result;		
	}

}

#include <bob_coverage/discrete_area_shapes.h>

#include <bob_toolbox/world_point_shapes.h>
#include <bob_toolbox/grid_algorithms.h>

namespace bob
{

	DiscreteArea discreteAreaCircle(WorldPoint center, float radius, float resolution)
	{
		std::vector<MapLocation> mask;	
		DiscreteArea circle(resolution);
		MapLocation discreteCenter = circle.worldToMap(center);		
		int discreteRadius = radius / resolution;

		for (int x = -discreteRadius; x <= discreteRadius; x++)
		{
			int circleHeight = (int)sqrt(pow(discreteRadius, 2) - pow((float)x, 2));
			for (int y = -circleHeight; y <= circleHeight; ++y)
			{
				mask.push_back(MapLocation(discreteCenter.x + x, discreteCenter.y + y));
			}
		}
		circle.insert(mask);
		return circle;
	}	

	DiscreteArea discreteAreaRectangle(WorldPoint center, float rotation, float width, float height, float resolution)
	{
		DiscreteArea rectangle(resolution);
		std::vector<WorldPoint> polygonOutline = worldPointRectangle(center, rotation, width, height);
		std::vector<MapLocation> discretePolygonOutline = rectangle.worldToMap< std::vector<MapLocation> >(polygonOutline);
		std::vector<MapLocation> filledCells = convexFillCells(discretePolygonOutline);
		rectangle.insert(filledCells);
		return rectangle;
	}

	DiscreteArea discreteAreaSquare(WorldPoint center, float rotation, float width, float resolution)
	{
		return discreteAreaRectangle(center, rotation, width, width, resolution);
	}

}


#include <bob_toolbox/world_point.h>

namespace bob
{
	WorldVector unitVector(float angle)
	{
		return WorldVector(cos(angle), sin(angle));
	}
	
	float dotProduct(const WorldVector& point1, const WorldVector& point2)
	{
		return ((point1.x * point2.x) + (point1.y * point2.y));
	}	

	WorldVector rotate(const WorldVector& point, float angle)
	{
		WorldPoint newPoint;
		newPoint.x = point.x * cos(angle) - point.y * sin(angle);
		newPoint.y = point.x * sin(angle) + point.y * cos(angle);
		return newPoint;
	}
}


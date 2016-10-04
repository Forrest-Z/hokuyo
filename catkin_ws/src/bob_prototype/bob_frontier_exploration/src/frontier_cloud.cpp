#include <bob_frontier_exploration/frontier_cloud.h>

#include <limits>
#include <vector>
#include <algorithm>

#include <bob_toolbox/world_point.h>
#include <bob_toolbox/geometry.h>

namespace bob
{

	MapLocation closestPoint(const FrontierCloud& cloud, MapLocation point)
	{
		float minDistance = std::numeric_limits<float>::max();
		MapLocation closestPoint;
		for (FrontierCloud::const_iterator itr = cloud.begin(); itr != cloud.end(); ++itr)
		{
			float newDistance = diagonalDistance(*itr, point);
			if (newDistance < minDistance)
			{
				minDistance = newDistance;
				closestPoint = *itr;
			}
		}
		return closestPoint;
	}
}

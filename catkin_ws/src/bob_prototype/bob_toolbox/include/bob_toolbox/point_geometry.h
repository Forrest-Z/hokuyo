#ifndef _BOB_TOOLBOX_POINT_GEOMETRY_H_
#define _BOB_TOOLBOX_POINT_GEOMETRY_H_

#include <vector>
#include <limits>

#include <bob_toolbox/world_point.h>
#include <bob_toolbox/geometry.h>

namespace bob
{

	//! Determines the closest point in a vector to another point
	template <typename Iterable, typename PointType>
	PointType closestPointTo(const Iterable& points, PointType point)
	{
		// Find closest point and set it to be the starting point
		float minDist = std::numeric_limits<float>::max();
		PointType closestPoint;
		for(typename Iterable::const_iterator itr = points.begin(); itr != points.end(); ++itr)
		{
			const PointType& currentPoint = *itr;
			float currentDistance = diagonalDistance(currentPoint, point);
			if(currentDistance < minDist)
			{	
				minDist = currentDistance;
				closestPoint = currentPoint;
			}
		}
		return closestPoint;
	}

}

#endif

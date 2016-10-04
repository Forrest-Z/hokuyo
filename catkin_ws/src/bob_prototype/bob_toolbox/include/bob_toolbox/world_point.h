#ifndef _BOB_TOOLBOX_WORLD_POINT_H_
#define _BOB_TOOLBOX_WORLD_POINT_H_

#include <cmath>
#include <bob_toolbox/generic_point.h>

namespace bob
{
	
	typedef GenericPoint<float> WorldPoint;
	typedef GenericVector<float> WorldVector;

	WorldVector unitVector(float angle);

	WorldVector rotate(const WorldVector& point, float angle);	

	float dotProduct(const WorldVector& point1, const WorldVector& point2);

}

#endif

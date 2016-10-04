#include <bob_toolbox/map_half_space.h>

#include <cmath>
#include <bob_toolbox/geometry.h>

namespace bob
{

	MapHalfSpace::MapHalfSpace(WorldPoint origin, float normalAngle) :
		origin(origin),
		normal(unitVector(normalAngle))
	{}

	bool MapHalfSpace::contains(WorldPoint point) const
	{
		WorldVector diff = point - origin;
		float dot = dotProduct(diff, normal);

		if (dot > 1e-8)
		{
			return true;
		}
		else
		{
			return false;
		}
	}


}

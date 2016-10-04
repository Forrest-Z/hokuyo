#ifndef _BOB_COVERAGE_DISCRETE_AREA_SHAPES_H_
#define _BOB_COVERAGE_DISCRETE_AREA_SHAPES_H_

#include <bob_coverage/discrete_area.h>

namespace bob
{

	DiscreteArea discreteAreaCircle(WorldPoint center, float radius, float resolution);

	DiscreteArea discreteAreaRectangle(WorldPoint center, float rotation, float width, float height, float resolution); 

	DiscreteArea discreteAreaSquare(WorldPoint center, float rotation, float width, float resolution);

}

#endif

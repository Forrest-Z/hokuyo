#ifndef _BOB_TOOLBOX_WORLD_POINT_SHAPES_H_
#define _BOB_TOOLBOX_WORLD_POINT_SHAPES_H_

#include <bob_toolbox/world_point.h>
#include <vector>

namespace bob
{

	std::vector<WorldPoint> worldPointRectangle(WorldPoint center, float rotation, float width, float height);

	std::vector<WorldPoint> worldPointSquare(WorldPoint center, float rotation, float width);

}

#endif

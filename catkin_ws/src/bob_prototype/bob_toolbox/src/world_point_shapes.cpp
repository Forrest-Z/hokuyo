#include <bob_toolbox/world_point_shapes.h>

#include <cmath>

namespace bob
{

	std::vector<WorldPoint> worldPointRectangle(WorldPoint center, float rotation, float width, float height)
	{
		std::vector<WorldPoint> result;

		float cosTheta = cos(rotation);
		float sinTheta = sin(rotation);
		float halfWidths[4] = {width/2, -width/2, -width/2, width/2};
		float halfHeights[4] = {height/2, height/2, -height/2, -height/2};

		// Loop that calculates the four vertices of square
		for (int i = 0; i <= 3; i++)
		{
			float x = center.x + halfWidths[i] * cosTheta - halfHeights[i] * sinTheta;
			float y = center.y + halfWidths[i] * sinTheta + halfHeights[i] * cosTheta;	

			result.push_back(WorldPoint(x, y));
		}	
		return result;
	}

	std::vector<WorldPoint> worldPointSquare(WorldPoint center, float rotation, float width)
	{
		return worldPointRectangle(center, rotation, width, width);
	}

}


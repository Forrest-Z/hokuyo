#ifndef _BOB_TOOLBOX_WORLD_RECTANGLE_H_
#define _BOB_TOOLBOX_WORLD_RECTANGLE_H_

#include <vector>
#include <cmath>
#include <bob_toolbox/world_point.h>
#include <bob_toolbox/dimension2d.h>

namespace bob 
{

	//! Represents a rectangle existing in the world.

	//! The bottom left corner of the rectangle is signified by an origin.
	//! The rectangle is shifted counter-clockwise about that origin by angle.
	//! The remaining points can be found using the defined bounds.
	struct WorldRectangle 
	{
	
		float angle;
		WorldVector origin;
		Dimension2D<float> bounds;	
	
		std::vector<WorldPoint> getPoints() const
		{
			std::vector<WorldPoint> toReturn;
			toReturn.push_back(origin + rotate(WorldPoint(0, 0), angle));						
			toReturn.push_back(origin + rotate(WorldPoint(bounds.width, 0), angle));						
			toReturn.push_back(origin + rotate(WorldPoint(bounds.width, bounds.height), angle));						
			toReturn.push_back(origin + rotate(WorldPoint(0, bounds.height), angle));
			return toReturn;						
		}

		WorldPoint getCenter() const
		{
			return origin + rotate(WorldVector(bounds.width / 2, bounds.height / 2), angle);
		}
		
		//! The area of the rectangle
		float getArea() const
		{
			return bounds.height * bounds.width;
		}
	};
}

#endif

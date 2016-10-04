#ifndef _BOB_TOOLBOX_RAYTRACE_H_
#define _BOB_TOOLBOX_RAYTRACE_H_

#include <bob_toolbox/axis.h>
#include <bob_toolbox/sign.h>

namespace bob
{

	//! Helper function used in bresenham2D
	template <class Cell>
		void incrementAxis(Cell& cell, Axis axis, int axisSign)
		{
			if (axis == x)
				cell.x += axisSign;
			if (axis == y)
				cell.y += axisSign; 
		}

	//! Performs the basics of the bresenham2D line algorithm after being initialized with some basic parameters
	template<class Cell>
		std::vector<Cell> bresenham2D(Cell start, Axis majorAxis, int majorDistance, int minorDistance, int error) 
		{
			Cell temp = start;
			std::vector<Cell> toReturn;

			//! Determine the minor axis
			Axis minorAxis = (majorAxis == x) ? y : x; 
	
			int majorSign = sign(majorDistance);
			int minorSign = sign(minorDistance);

			for (unsigned int majorIndex = 0; majorIndex < (unsigned int)abs(majorDistance); ++majorIndex)
			{
				//! Store the current cell before calculating the next
				toReturn.push_back(temp);

				//! Increment in the direction of the major axis
				incrementAxis(temp, majorAxis, majorSign);
				error += abs(minorDistance);

				//! Increment in the direction of the minor axis, if necessary
				if ((unsigned int)error >= abs(majorDistance))
				{
					incrementAxis(temp, minorAxis, minorSign);
					error -= abs(majorDistance);
				}
			}

			//! Add the last cell too
			toReturn.push_back(temp);
			return toReturn;
		}

	//! Generates all the cells resulting from tracing a ray between two cells

	//! Uses the bresenham algorithm as in the bresenham2D function
	template<class Cell>
		std::vector<Cell> raytraceLine(Cell from, Cell to) 
		{
			int dx = to.x - from.x;
			int dy = to.y - from.y;

			int error;
			int majorDistance;
			int minorDistance;
			unsigned int maxDistance;
			Axis majorAxis;
			if (abs(dx) >= abs(dy))
			{
				//! x is major axis
				error = abs(dx / 2);
				majorDistance = dx;
				minorDistance = dy;	
				majorAxis = x;
			}
			else
			{
				//! y is major axis
				error = abs(dy / 2);
				majorDistance = dy;
				minorDistance = dx;
				majorAxis = y;
			}
			return bresenham2D(from, majorAxis, majorDistance, minorDistance, error);
		}

}

#endif

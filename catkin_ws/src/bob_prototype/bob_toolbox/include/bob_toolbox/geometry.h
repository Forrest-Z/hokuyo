#ifndef _BOB_TOOLBOX_GEOMETRY_H_
#define _BOB_TOOLBOX_GEOMETRY_H_

#include <cmath>
#include <algorithm>

namespace bob
{

	//! \brief The angle that a ray of two points make from parallel to the x axis
	//! \param origin The origin of the ray
	//! \param arrowHead The head of the ray
	//! \return The angle the ray makes with the parallel to the x axis
	template <typename Point>
	float rayAngle(const Point& origin, const Point& arrowHead)
	{
		float yDiff = (float)(arrowHead.y - origin.y);
		float xDiff = (float)(arrowHead.x - origin.x);
		return atan2(yDiff, xDiff);	
	}

	//! \brief Calculates the distance between two points
	//! \param first A point
	//! \param second A point
	//! \return The distance between the two points
	template <typename Point>
	float diagonalDistance(const Point& to, const Point& from)
	{
		return (from - to).getLength();
	}

	//! \brief Computes a result from the taking the min of each element in the inputs
	//! \param first A point
	//! \param second A point
	//! \return A new point defined by (min(first.x, second.x), min(first.y, second.y))
	template <typename Point>
	Point elementalMin(const Point& first, const Point& second)
	{
		Point newPoint;
		newPoint.x = std::min(first.x, second.x);
		newPoint.y = std::min(first.y, second.y);
		return newPoint;	
	}

	//! \brief Computes a result from the taking the max of each element in the inputs
	//! \param first A point
	//! \param second A point
	//! \return A new point defined by (max(first.x, second.x), max(first.y, second.y))
	template <typename Point>
	Point elementalMax(const Point& first, const Point& second)
	{
		Point newPoint;
		newPoint.x = std::max(first.x, second.x);
		newPoint.y = std::max(first.y, second.y);
		return newPoint;	
	}
}

#endif

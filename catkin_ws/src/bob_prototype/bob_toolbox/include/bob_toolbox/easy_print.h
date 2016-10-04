#ifndef _BOB_TOOLBOX_EASY_PRINT_H_
#define _BOB_TOOLBOX_EASY_PRINT_H_

#include <iostream>
#include <bob_toolbox/pose2d.h>
#include <bob_toolbox/angular_range.h>
#include <bob_toolbox/line.h>
#include <bob_toolbox/matrix3.h>

namespace bob
{

	std::ostream& operator<<(std::ostream& os, const Pose2D& pose);

	template <typename ListDataType>
	std::ostream& operator<<(std::ostream& os, const std::list<ListDataType> iterable)
	{
		os << "{ ";
		for (typename std::list<ListDataType>::const_iterator itr = iterable.begin(); itr != iterable.end(); ++itr)
		{
			os << *itr;	
			if (itr != --(iterable.end()))
				os << ", ";
		}
		os << "}";
		return os;
	}

	template <typename PointDataType>
	std::ostream& operator<<(std::ostream& os, const GenericPoint<PointDataType>& point)
	{
		os << "(" << point.x << "," << point.y << ")";
		return os;
	}

	template <typename PointDataType>
	std::ostream& operator<<(std::ostream& os, const GenericVector<PointDataType>& point)
	{
		os << "(" << point.x << "," << point.y << ")";
		return os;
	}

	std::ostream& operator<<(std::ostream& os, const AngularRange& range);

	std::ostream& operator<<(std::ostream& os, const SimpleAngularRange& range);

	std::ostream& operator<<(std::ostream& os, const Line& range);

	std::ostream& operator<<(std::ostream& os, const Matrix3& matrix);
}

#endif

#include <bob_toolbox/easy_print.h>

namespace bob
{

	std::ostream& operator<<(std::ostream& os, const Pose2D& pose)
	{
		os << "(" << pose.x << ", " << pose.y << ", " << pose.theta << ")";
		return os;
	}

	/*
	std::ostream& operator<<(std::ostream& os, const WorldPoint& worldPoint)
	{
		os << "(" << worldPoint.x << "," << worldPoint.y << ")";
		return os;
	}
	*/

	std::ostream& operator<<(std::ostream& os, const AngularRange& range)
	{
		os << "Angular Range:\n";
		AngularRange::container ranges = range.getRanges();
		for (AngularRange::container::const_iterator itr = ranges.begin();
			itr != ranges.end();
			++itr)
		{
			os << "Simple Range: " << *itr << "\n";
		}
		return os;
	}

	std::ostream& operator<<(std::ostream& os, const SimpleAngularRange& range)
	{
		os << "( " << range.lower << ", " << range.upper << ")";
		return os;
	}

	std::ostream& operator<<(std::ostream& os, const Line& range)
	{
		os << "(" << range.origin << " + k" << range.vector << "')";
		return os;
	}

	std::ostream& operator<<(std::ostream& os, const Matrix3& matrix)
	{
		os <<  "[" << matrix(0,0) << ", " << matrix(0,1) << ", " << matrix(0,2) << "\n"
			   << matrix(1,0) << ", " << matrix(1,1) << ", " << matrix(1,2) << "\n"
			   << matrix(2,0) << ", " << matrix(2,1) << ", " << matrix(2,2) << "]\n";
		return os;
	}
}


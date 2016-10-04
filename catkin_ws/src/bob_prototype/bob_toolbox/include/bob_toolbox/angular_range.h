#ifndef _BOB_TOOLBOX_ANGULAR_RANGE_H_
#define _BOB_TOOLBOX_ANGULAR_RANGE_H_

#include <list>
#include <bob_toolbox/angles.h>

namespace bob
{

	//! Defines an basic angular range defined by a lower and upper bound.

	//! The angle are normalized on input.
	//! The range is considered in the counterclockwise order,
	//! [lower, upper]
	//! If lower == upper the range is considered to be [0, 2pi],
	//! (the full circle)
	//! Note: This means that discrete angles CANNOT be stored.
	//! ie. [0.5, 0.5] cannot be stored as 0.5 and will be interpreted
	//! as a full circle.
	struct SimpleAngularRange
	{

		//! Construct a simple angular range with given bounds
		SimpleAngularRange(float lower, float upper) :
			lower(normalizeAnglePos(lower)),
			upper(normalizeAnglePos(upper))
		{}	

		float lower, upper;

		//! Checks if an angle is contained inside the range
		bool isInside(float angle) const;		

		//! Detects if two ranges overlap with each other
		bool overlapsWith(SimpleAngularRange rhs) const;

		//! Checks if the range is a full 2pi circle
		bool isFullCircle() const;

		//! Clip the range so that it lies within another range
		//! \return True if the resulting SimpleAngularRange is valid, false if it
		//! is invalid because clipping completely erased the simple angular range
		bool limitBy(const SimpleAngularRange& limit);

	};

	//! Defines a composite angular range, made up of several SimpleAngularRanges

	//! The code for this class relies on the fact that the container's
	//! order is consistent with the physical meaning of the order.
	class AngularRange
	{

		public:

			typedef std::list<SimpleAngularRange> container;

			//! Gets all the ranges
			inline container getRanges() const
			{
				return data;
			}
	
			//! Checks if a particular angle is contained in the range
			bool isInside(float angle) const;

			//! Inserts new simple ranges into the AngularRange object.

			//! Is designed to keep the data consistent so there are no
			//! overlapping SimpleAngularRanges in the object.
			//! Also keeps the order of the ranges.
			void add(SimpleAngularRange range);

			//! Returns an inverted copy of the range
			AngularRange invert() const;

			//! Find the subrange that contains the given value
			SimpleAngularRange containingSubrange(float angle) const;

		private:

			//! Checks if two angular ranges create a full 2*pi circle
			bool createsFullCircle(SimpleAngularRange range1, SimpleAngularRange range2);

			container data; //!< The SimpleAngularRanges are stored here. The order of the elements matters
	

	};

}

#endif

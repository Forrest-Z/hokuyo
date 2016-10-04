#ifndef _BOB_TOOLBOX_LINEAR_RANGE_H_
#define _BOB_TOOLBOX_LINEAR_RANGE_H_

#include <algorithm>

namespace bob
{

	template <typename RangeType>
		class LinearRange
		{

			public:
				LinearRange(RangeType lower, RangeType upper) :
					lower(lower),
					upper(upper)
			{}

				RangeType limit(RangeType val)
				{
					val = std::max(val, lower);
					val = std::min(val, upper);
					return val;	
				}

			private:

				RangeType lower;
				RangeType upper;

		};

}

#endif

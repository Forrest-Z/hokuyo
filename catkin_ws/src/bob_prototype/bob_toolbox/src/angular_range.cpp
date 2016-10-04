#include <bob_toolbox/angular_range.h>

#include <bob_toolbox/logging.h>
#include <iterator>

namespace bob
{

	bool SimpleAngularRange::limitBy(const SimpleAngularRange& limit)
	{
		bool lowerInside = limit.isInside(lower);	
		bool upperInside = limit.isInside(upper);

		if (lowerInside)
		{
			if (upperInside)
			{
				// Both inside, so we don't need to modify the range
				return true;
			}
			else
			{
				// Lower inside, upper not
				upper = limit.upper;
				return true;
			}
		}
		else
		{
			if (upperInside)
			{
				// Upper inside, lower not		
				lower = limit.lower;
				return true;
			}
			else
			{
				// Neither inside
				if (isInside(limit.lower))
				{
					// The limit is completely inside
					// the range
					lower = limit.lower;
					upper = limit.upper;
					return true;
				}
				else
				{
					// The limit is completely outside
					// the range
					return false;
				}
			}
		}
	}
	
	bool SimpleAngularRange::isInside(float angle) const
	{
		// Full circle contains all angles
		if (isFullCircle())
			return true;
		
		float offset = normalizeAnglePos(normalizeAnglePos(angle) - lower);
		float rangeSize = normalizeAnglePos(upper - lower);
		return (offset <= rangeSize);
	}

	bool SimpleAngularRange::overlapsWith(SimpleAngularRange rhs) const
	{
		return (isInside(rhs.lower) || isInside(rhs.upper) ||
			rhs.isInside(lower) || rhs.isInside(upper));
	}

	bool SimpleAngularRange::isFullCircle() const
	{
		return (fabs(lower - upper) < 0.0001);
	}

	void AngularRange::add(SimpleAngularRange range)
	{
		// Where the range while eventually be inserted into the container
		container::iterator whereToInsert = data.end();

		for (container::iterator itr = data.begin(); itr != data.end();)
		{
			if (createsFullCircle(range, *itr))
			{
				data.clear();
				data.push_back(SimpleAngularRange(0, 0));
				return;
			}
			else if (range.overlapsWith(*itr))
			{
				// The new range overlaps with one of the ranges
				// already in the container. Therefore, we will
				// adjust the size and then replace the existing range
				// with the new one.
				if (itr->isInside(range.lower))
				{
					range.lower = itr->lower;
				}
					
				if (itr->isInside(range.upper))
				{
					range.upper = itr->upper;
				}	
				
				// Remove the existing range because we will replace
				// it with the new one.
				whereToInsert = itr = data.erase(itr);
			}
			else
			{
				// Also need to check if the range fits in between existing
				// ranges, so we can insert it there if necessary.
				container::iterator nextElement = itr;
				++nextElement;
				
				if (nextElement != data.end())
				{
					// Note: We don't need to handle the "else" case because whereToInsert is 
					// initialized with data.end()
					if (SimpleAngularRange(itr->upper, nextElement->lower).overlapsWith(range))	
					{
						// The new range exists between two existing ranges, so record the
						// position of where to insert it
						whereToInsert = nextElement;
					}
				}
				else
				{
					if(range.upper < itr->lower && range.lower < itr->lower)
					{
						whereToInsert = data.begin();
					}
				}

				// The itr needs to be inrementedthis way because 
				// we are erasing while iterating
				++itr;
			}
		}
		data.insert(whereToInsert, range);
	}

	bool AngularRange::isInside(float angle) const
	{
		for (container::const_iterator itr = data.begin(); itr != data.end(); ++itr)
		{
			if (itr->isInside(angle))
				return true;
		}
		return false;
	}

	AngularRange AngularRange::invert() const
	{

		// This logic relies on the fact that the container is correctly ordered!
		AngularRange toReturn;

		if (data.size() == 0)
		{
			toReturn.add(SimpleAngularRange(0, 0));		
			return toReturn;
		}

		// Two iterators are needed because list doesn't allow stuff like (itr + 1)		
		container::const_iterator itrUpper = data.begin();
		container::const_iterator itrLower = data.begin();
		++itrLower;

		for (; itrLower != data.end(); ++itrUpper, ++itrLower)
		{
			SimpleAngularRange newRange(itrUpper->upper, itrLower->lower);	
			toReturn.add(newRange);
		}
		SimpleAngularRange newRange((--data.end())->upper, data.begin()->lower);	
		toReturn.add(newRange);
		return toReturn;
	}

	SimpleAngularRange AngularRange::containingSubrange(float angle) const
	{
		for (container::const_iterator itr = data.begin(); itr != data.end(); ++itr)
		{
			if (itr->isInside(angle))
				return *itr;
		} 
		LOG_ERROR("Calling containingSubrange but no containing element found. You should call 'isInside' first");
	}

	bool AngularRange::createsFullCircle(SimpleAngularRange range1, SimpleAngularRange range2)
	{
		// Hard to explain how this works. Try drawing it on paper.
		return (range1.isInside(range2.lower) && range1.isInside(range2.upper) && 
			range2.isInside(range1.lower) && range2.isInside(range1.upper));
	}
}


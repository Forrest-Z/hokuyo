#ifndef _BOB_TOOLBOX_SET_ALGORITHMS_H_
#define _BOB_TOOLBOX_SET_ALGORITHMS_H_

#include <unordered_set>
#include <limits.h>

namespace bob
{

	//! This function is similar to std::remove_if, which cannot be used on unordered_sets
	//! Elements from the set are removed if they satisfy the functor
	template <class DataType, class HashFcn, class Functor>
	void removeFromSetIf(std::unordered_set<DataType, HashFcn>& data, const Functor& functor)
	{
		for (auto itr = data.begin(); itr != data.end();)
		{
			if (functor(*itr))
			{
				itr = data.erase(itr); //! This increments the itr
			}
			else
			{	
				++itr;
			}

		}
	}

	template <class DataType, class HashFcn, class Functor>
	void removeFromSetIf(std::unordered_set<DataType, HashFcn>& data, Functor& functor, std::unordered_set<DataType>& removeTo)
	{
		for (auto itr = data.begin(); itr != data.end();)
		{
			if (functor(*itr))
			{
				if (removeTo.count(*itr) == 0)
				{
					removeTo.insert(*itr);
				}

				itr = data.erase(itr); //! This increments the itr
			}
			else
			{	
				++itr;
			}

		}
	}

	template <class DataType, class HashFcn>
	DataType closestPointTo(const std::unordered_set<DataType, HashFcn>& data, const DataType& origin)
	{
		float minDistance = std::numeric_limits<float>::max();
		DataType closestPoint;

		for (auto itr = data.begin(); itr != data.end(); ++itr)
		{
			double distance = diagonalDistance(origin, *itr); 
			if (distance < minDistance)
			{	
				minDistance = distance;
				closestPoint = *itr;				
			}
		}
		
		return closestPoint;
	}

}

#endif

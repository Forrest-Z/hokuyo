#ifndef _BOB_GRID_MAP_DISTANCE_CACHE_MATRIX_H_
#define _BOB_GRID_MAP_DISTANCE_CACHE_MATRIX_H_

#include <bob_toolbox/int_point.h>

#include <bob_toolbox/array2d.h>

namespace bob
{

	//! \brief A matrix of distance values, which are cached when instances are created.
	//! The length of IntVector objects can then be retrieved using the provided member function.
	//! The purpose of this class is to avoid having to recalculate hypot(x, y), which can be
	//! computationally expensive. With this class hypot(x, y) is calculated up-front for a matrix
	//! of possible IntVector values, and cached for later use. 
	class DistanceCacheMatrix
	{

		public:

			//! \brief Construct an instance of DistanceCacheMatrix
			//! \param distanceBound A lower bound on the distance that can be looked up using the cache.
			//! Any distance lower than this value is guarantetd to be found in the cache.
			//! \param resolution The resolution of the cells in the IntVector.
			DistanceCacheMatrix(float distanceBound, float resolution);

			//! \brief Looks up the length of an IntVector using the distance cache
			//! \param toMeasure The vector to measure
			//! \param[out] distance Output variable for the resulting distance
			//! \return True if lookup was successful and IntVector is in table
			bool distanceLookup(IntVector toMeasure, float& distance);

		private:

			//! Underlying data storage
			Array2D<float> data;

	};

}

#endif

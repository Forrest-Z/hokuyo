#include <bob_grid_map/distance_cache_matrix.h>

#include <cmath>

namespace bob
{

	DistanceCacheMatrix::DistanceCacheMatrix(float distanceBound, float resolution)
	{
		int intWidth = (int)ceil(distanceBound / resolution);
		Dimension2D<int> matrixDimension(intWidth, intWidth);
		data = Array2D<float>(matrixDimension);

		for (unsigned int i = 0; i < intWidth; ++i)
		{
			for (unsigned int j = 0; j < intWidth; ++j)
			{
				data[Array2D<float>::Point(i, j)] = resolution * hypot(i, j);
			}
		}

	}

	bool DistanceCacheMatrix::distanceLookup(IntVector toMeasure, float& distance)
	{
		Array2D<float>::Point point(toMeasure);
		point.x = std::abs(point.x);
		point.y = std::abs(point.y);
		if (!data.isInside(point))
		{
			return false;
		}
		else
		{
			distance = data[point];
			return true;
		}
	}
}


#include <bob_toolbox/basic_algebra.h>

#include <cmath>

namespace bob
{

	QuadraticSolution quadraticFactor(float a, float b, float c)
	{
		QuadraticSolution toReturn;
		float determinant = pow(b, 2) - 4 * a * c;
		float firstPart = - b / (2 * a);
		float sqrtDeterminant = 0;
		if (determinant < 0)
		{
			sqrtDeterminant = sqrt(-determinant);
			float dividedDeterminant = sqrtDeterminant / (2 * a);
			toReturn.first = FloatComplex(firstPart, dividedDeterminant);
			toReturn.second = FloatComplex(firstPart, -dividedDeterminant);
		}
		else
		{
			sqrtDeterminant = sqrt(determinant);
			float dividedDeterminant = sqrtDeterminant / (2 * a);
			toReturn.first = FloatComplex(firstPart + dividedDeterminant, 0);
			toReturn.second = FloatComplex(firstPart - dividedDeterminant, 0);
		}
		return toReturn;
	}

}


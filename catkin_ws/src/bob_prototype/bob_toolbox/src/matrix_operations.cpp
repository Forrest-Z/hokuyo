#include <bob_toolbox/matrix_operations.h>

#include <bob_toolbox/basic_algebra.h>
#include <bob_toolbox/logging.h>

namespace bob
{

	Vector3f operator*(const Matrix3& matrix, const Vector3f& vector)
	{
		Vector3f toReturn = Vector3f::zeros();
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
				toReturn(i) += matrix(j, i) * vector(j);
		}
		return toReturn;
	}

	bool inverse(const Matrix3& input, Matrix3& output)
	{
		float det = determinate(input);

		if (det == 0)
			return false;

		output = adjoint(input) / det;
		return true;
	}

	Matrix3 adjoint(const Matrix3& input)
	{
		return transpose(cofactor(input)); 
	}

	Matrix3 cofactor(const Matrix3& input)
	{
		Matrix3 toReturn;
		toReturn(0, 0) = input(1, 1) * input(2, 2) - 
				 input(2, 1) * input(1, 2);

		toReturn(1, 0) = input(2, 1) * input(0, 2) - 
				 input(0, 1) * input(2, 2);

		toReturn(2, 0) = input(0, 1) * input(1, 2) - 
				 input(1, 1) * input(0, 2);

		toReturn(0, 1) = input(1, 2) * input(2, 0) - 
				 input(2, 2) * input(1, 0);

		toReturn(1, 1) = input(2, 2) * input(0, 0) - 
				 input(0, 2) * input(2, 0);

		toReturn(2, 1) = input(0, 2) * input(1, 0) - 
				 input(1, 2) * input(0, 0);

		toReturn(0, 2) = input(1, 0) * input(2, 1) - 
				 input(2, 0) * input(1, 1);

		toReturn(1, 2) = input(2, 0) * input(0, 1) - 
				 input(0, 0) * input(2, 1);

		toReturn(2, 2) = input(0, 0) * input(1, 1) - 
				 input(1, 0) * input(0, 1);
		return toReturn;
	}

	Matrix3 transpose(const Matrix3& input)
	{
		Matrix3 toReturn;
		toReturn(0, 0) = input(0, 0);
		toReturn(1, 1) = input(1, 1);
		toReturn(2, 2) = input(2, 2);

		toReturn(0, 1) = input(1, 0);
		toReturn(0, 2) = input(2, 0);
		toReturn(1, 2) = input(2, 1);

		toReturn(1, 0) = input(0, 1);
		toReturn(2, 0) = input(0, 2);
		toReturn(2, 1) = input(1, 2);
		return toReturn;
	}

	float determinate(const Matrix3& input)
	{
		return input(0, 0) * (input(1, 1) * input(2, 2) -
					input(1, 2) * input(2, 1))
			+ input(1, 0) * (input(2, 1) * input(0, 2) -
					input(2, 2) * input(0, 1))
			+ input(2, 0) * (input(0, 1) * input(1, 2) -
					input(0, 2) * input(1, 1));
	}

	float determinant(const Matrix2& input)
	{
		return input(0, 0) * input(1, 1) - input(1, 0) * input(0, 1);
	}

	Matrix2 adjoint(const Matrix2& input)
	{
		Matrix2 toReturn;
		toReturn(0, 0) = input(1, 1);
		toReturn(1, 1) = input(0, 0);
		toReturn(1, 0) = -input(1, 0);
		toReturn(0, 1) = -input(0, 1);
		return toReturn;
	}

	bool inverse(const Matrix2& input, Matrix2& output)
	{
		float det = determinant(input);
		
		if (det == 0)
			return false;

		output = adjoint(input) / det;
		return true;
	}

	bool isDiagonalMatrix(const Matrix2& input)
	{
		return input(0, 1) == 0 && input(1, 0) == 0;
	}

	EigenValueSolution eigenPairs(const Matrix2& input)
	{
		EigenValueSolution toReturn;

		if (isDiagonalMatrix(input))
		{
			toReturn.first.value = input(0, 0);
			toReturn.second.value = input(1, 1);
			toReturn.first.vector.x = 1;
			toReturn.first.vector.y = 0;
			toReturn.second.vector.x = 0;
			toReturn.second.vector.y = 1;
			
			return toReturn;
		}

		float a = 1;
		float b = -(input(0, 0) + input(1, 1));
		float c = determinant(input);

		QuadraticSolution eigenValues = quadraticFactor(a, b, c); 

		if (eigenValues.first.imag() != 0 || eigenValues.second.imag() != 0)
			LOG_ERROR("Zero eigenvalues obtained, something must be wrong!")	

		toReturn.first.value = eigenValues.first.real();
		toReturn.second.value = eigenValues.second.real();
		toReturn.first.vector.x = 1;
		toReturn.second.vector.x = 1;
		
		toReturn.first.vector.y = (toReturn.first.value - input(0, 0)) / input(1, 0);
		toReturn.second.vector.y = (toReturn.second.value - input(0, 0)) / input(1, 0);

		toReturn.first.vector = toReturn.first.vector / toReturn.first.vector.getLength();
		toReturn.second.vector = toReturn.second.vector / toReturn.second.vector.getLength();

		return toReturn;
			
	}

}


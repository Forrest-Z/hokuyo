#ifndef _BOB_TOOLBOX_MATRIX_OPERATIONS_H_
#define _BOB_TOOLBOX_MATRIX_OPERATIONS_H_

#include <bob_toolbox/matrix3.h>
#include <bob_toolbox/matrix2.h>
#include <bob_toolbox/vector3.h>
#include <bob_toolbox/eigen_pair.h>

#include <utility>

namespace bob
{

	typedef std::pair<EigenPair, EigenPair> EigenValueSolution;

	//! Take the inverse of a matrix.

	//! @param[in] input The matrix to invert
	//! @param[out] output The result of the inversion
	//! @return Returns true if the matrix is invertible. Returns false if singular.
	bool inverse(const Matrix3& input, Matrix3& output);

	//! Calculate the adjoint of a 3x3 matrix.
	Matrix3 adjoint(const Matrix3& input);

	//! Calculate the determinate of a 3x3 matrix.
	float determinate(const Matrix3& input);

	//! Calculate the "cofactor matrix" of a 3x3 matrix.
	Matrix3 cofactor(const Matrix3& input);

	//! Calculate the transpose of a 3x3 matrix.
	Matrix3 transpose(const Matrix3& input);

	//! Operator used to multiply a matrix by a vector
	Vector3f operator*(const Matrix3& matrix, const Vector3f& vector);

	//! Calculate the determinant of a 2x2 matrix
	float determinant(const Matrix2& input);
	
	//! Calculate the inverse of a 2x3 matrix
	bool inverse(const Matrix2& input, Matrix2& output);

	//! Calculate the adjoint of a 2x2 matrix
	Matrix2 adjoint(const Matrix2& input);

	EigenValueSolution eigenPairs(const Matrix2& input);

	template <typename Matrix6>
	Matrix3 toMatrix3(Matrix6 source, int startIdx = 0)
	{
		Matrix3 toReturn;
		toReturn(0, 0) = source(startIdx, startIdx);
		toReturn(0, 1) = source(startIdx, startIdx + 1);
		toReturn(0, 2) = source(startIdx, startIdx + 5);
		toReturn(1, 0) = source(startIdx + 1, startIdx);
		toReturn(1, 2) = source(startIdx + 1, startIdx + 1);
		toReturn(1, 3) = source(startIdx + 1, startIdx + 5);
		toReturn(2, 0) = source(startIdx + 2, startIdx);
		toReturn(2, 1) = source(startIdx + 2, startIdx + 1);
		toReturn(2, 2) = source(startIdx + 2, startIdx + 5);
		
		return toReturn;

	} 

}

#endif

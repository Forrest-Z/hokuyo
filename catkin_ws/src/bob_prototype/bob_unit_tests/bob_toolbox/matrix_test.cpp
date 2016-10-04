#include <gtest/gtest.h>
#include <bob_toolbox/angular_range.h>
#include <bob_toolbox/angles.h>
#include <bob_toolbox/matrix3.h>
#include <bob_toolbox/matrix2.h>
#include <bob_toolbox/matrix_operations.h>
#include <bob_toolbox/logging.h>

using namespace bob;

// ASSERT_NEAR
// ASSERT_TRUE
TEST (MatrixTest, DiagonalMatrixTest)
{
	Matrix2 input = Matrix2::zeros();
	input(0, 0) = 1;	
	input(1, 1) = 2;	
	
	EigenValueSolution solution = eigenPairs(input);

	LOG_TEST("eigenvalue: " << solution.first.value	<< " eigenvector: " << solution.first.vector << "\n"); 
	LOG_TEST("eigenvalue: " << solution.second.value << " eigenvector: " << solution.second.vector << "\n"); 
	
}

//! Tests the adjoint() function
TEST (MatrixTest, AdjointTest)
{
	Matrix3 input;
	input(0, 0) = 1;
	input(1, 0) = 2;
	input(2, 0) = 3;

	input(0, 1) = 0;
	input(1, 1) = 4;
	input(2, 1) = 5;

	input(0, 2) = 1;
	input(1, 2) = 0;
	input(2, 2) = 6;

	Matrix3 output = adjoint(input);

	ASSERT_EQ(output(0, 0), 24);
	ASSERT_EQ(output(1, 0), -12);
	ASSERT_EQ(output(2, 0), -2);

	ASSERT_EQ(output(0, 1), 5);
	ASSERT_EQ(output(1, 1), 3);
	ASSERT_EQ(output(2, 1), -5);

	ASSERT_EQ(output(0, 2), -4);
	ASSERT_EQ(output(1, 2), 2);
	ASSERT_EQ(output(2, 2), 4);

}

TEST (MatrixTest, DeterminateTest)
{
	Matrix3 input;
	input(0, 0) = 3;
	input(1, 0) = -5;
	input(2, 0) = 3;

	input(0, 1) = 2;
	input(1, 1) = 1;
	input(2, 1) = -1;

	input(0, 2) = 1;
	input(1, 2) = 0;
	input(2, 2) = 4;

	float det = determinate(input);

	ASSERT_EQ(det, 54);

}

//! Tests the inverse() function
TEST (MatrixTest, InverseTest)
{
	Matrix3 input;
	input(0, 0) = 7;
	input(1, 0) = 2;
	input(2, 0) = 1;

	input(0, 1) = 0;
	input(1, 1) = 3;
	input(2, 1) = -1;

	input(0, 2) = -3;
	input(1, 2) = 4;
	input(2, 2) = -2;

	Matrix3 output;
 	inverse(input, output);

	ASSERT_EQ(output(0, 0), -2);
	ASSERT_EQ(output(1, 0), 8);
	ASSERT_EQ(output(2, 0), -5);

	ASSERT_EQ(output(0, 1), 3);
	ASSERT_EQ(output(1, 1), -11);
	ASSERT_EQ(output(2, 1), 7);

	ASSERT_EQ(output(0, 2), 9);
	ASSERT_EQ(output(1, 2), -34);
	ASSERT_EQ(output(2, 2), 21);
}

TEST (MatrixTest, CopyConstructor)
{
	Matrix3 first;
	first(0, 0) = 7;

	Matrix3 second(first);

	ASSERT_EQ(second(0, 0), 7);

	first(0, 0) = 5;

	ASSERT_EQ(second(0, 0), 7);
}

TEST (MatrixTest, AssignmentOperator)
{
	Matrix3 first;
	first(0, 0) = 7;

	Matrix3 second = first;

	ASSERT_EQ(second(0, 0), 7);

	first(0, 0) = 5;

	ASSERT_EQ(second(0, 0), 7);
	ASSERT_EQ(first(0, 0), 5);
}

//! Tests multiplication operator
TEST (MatrixTest, Multiplication)
{
	Matrix3 matrix;
	matrix(0, 0) = 2;
	matrix(1, 0) = 4;
	matrix(2, 0) = 5;

	matrix(0, 1) = 0;
	matrix(1, 1) = 1;
	matrix(2, 1) = 2;

	matrix(0, 2) = 3;
	matrix(1, 2) = 4;
	matrix(2, 2) = 2;

	Vector3f vector;
	vector(0) = 1;
	vector(1) = 2;
	vector(2) = 4;

	Vector3f testOutput = matrix * vector;
	
	ASSERT_EQ(testOutput(0), 30);
	ASSERT_EQ(testOutput(1), 10);
	ASSERT_EQ(testOutput(2), 19);	
}

TEST (MatrixTest, ZeroFactories)
{
	Matrix3 matrix = Matrix3::zeros();
	
	ASSERT_EQ(matrix(0, 0), 0.0);
	ASSERT_EQ(matrix(1, 0), 0.0);
	ASSERT_EQ(matrix(2, 0), 0.0);
	ASSERT_EQ(matrix(0, 1), 0.0);
	ASSERT_EQ(matrix(1, 1), 0.0);
	ASSERT_EQ(matrix(2, 1), 0.0);
	ASSERT_EQ(matrix(0, 2), 0.0);
	ASSERT_EQ(matrix(1, 2), 0.0);
	ASSERT_EQ(matrix(2, 2), 0.0);

	Vector3f vector3 = Vector3f::zeros();
	ASSERT_EQ(vector3(0), 0.0);
	ASSERT_EQ(vector3(1), 0.0);
	ASSERT_EQ(vector3(2), 0.0);	
}

TEST (MatrixTest, Eigen2D)
{
	// The matrix:
	// [[ 0  1 ],
	//  [-2 -3 ]]
	Matrix2 matrix;
	matrix(0, 0) = 0;
	matrix(1, 0) = 1;
	matrix(0, 1) = -2;
	matrix(1, 1) = -3;	

	EigenValueSolution eigenValueSolution = eigenPairs(matrix);

	// This is just sorting them so we can compare directly
	float smallerEigen = std::min(eigenValueSolution.first.value, eigenValueSolution.second.value);
	float largerEigen = std::max(eigenValueSolution.first.value, eigenValueSolution.second.value);
		
	// Eigenvalues should be -1 and -2
	ASSERT_NEAR(smallerEigen, -2.0, 0.001);	
	ASSERT_NEAR(largerEigen, -1.0, 0.001);	

	// The actual values for the eigen vectors are unimportant. Only the ratio matters 
	float firstRatio = eigenValueSolution.first.vector.x / eigenValueSolution.first.vector.y;
	float secondRatio = eigenValueSolution.second.vector.x / eigenValueSolution.second.vector.y;

	// Sort so we can compare directly
	float smallerRatio = std::min(firstRatio, secondRatio);
	float largerRatio = std::max(firstRatio, secondRatio);

	// Eigenvectors should be:
	// [ 1  -1 ]' and [ 1  -2 ]' 
	ASSERT_NEAR(smallerRatio, -1.0, 0.001);
	ASSERT_NEAR(largerRatio, -0.5, 0.001);


}



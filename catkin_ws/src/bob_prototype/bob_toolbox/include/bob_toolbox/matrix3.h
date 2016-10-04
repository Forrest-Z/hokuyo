#ifndef _BOB_TOOLBOX_MATRIX3_H_
#define _BOB_TOOLBOX_MATRIX3_H_

namespace bob
{

	//! Simple 3x3 matrix class. Supports only a few operations. More can be added as needed.
	class Matrix3
	{

		public:

			//! Creates a matrix of all zeros
			inline static Matrix3 zeros()
			{
				Matrix3 result;
				for (int i = 0; i < 9; i++)
					result.values[i] = 0.0;
				return result;
			}

			//! Gives access to array elements. eg matrix(0, 1) will access the first row and second column.
			inline float& operator()(int x, int y)
			{
				// Don't change this implementation! Change the const version.
				return const_cast<float&>(static_cast<const Matrix3*>(this)->operator()(x, y));
			}

			//! Const version of accessor
			inline const float& operator()(int x, int y) const
			{
				return values[3 * y + x];
			}

			//! Operator for dividing the elements of a matrix by a divisor
			Matrix3 operator/(const float& divisor) const
			{
				Matrix3 toReturn;
				for (int i = 0; i < 9; i++)
					toReturn.values[i] = values[i] / divisor;
				return toReturn;
			}

		private:

			//! Data storage
			float values[9];

	};

}

#endif

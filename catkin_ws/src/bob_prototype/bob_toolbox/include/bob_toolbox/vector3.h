#ifndef _BOB_TOOLBOX_VECTOR3_H_
#define _BOB_TOOLBOX_VECTOR3_H_

namespace bob
{

	//! Simple 3x1 vector class. This class defines only the most basic operations. More may be
	//! added in the future, as needed.
	class Vector3f
	{

		public: 

			//! Creates a vector of all zeros
			inline static Vector3f zeros()
			{
				Vector3f toReturn;
				for (int i = 0; i < 3; i++)
					toReturn.values[i] = 0;
				return toReturn;	
			}

			//! Allows access to vector elements
			inline float& operator()(int x)
			{
				// Do not change implementation! Change the const one.
				return const_cast<float&>(static_cast<const Vector3f*>(this)->operator()(x));
			}

			//! Const accessor
			inline const float& operator()(int x) const
			{
				return values[x];
			}
			

		private:

			//! Data storage
			float values[3];

	};

}

#endif

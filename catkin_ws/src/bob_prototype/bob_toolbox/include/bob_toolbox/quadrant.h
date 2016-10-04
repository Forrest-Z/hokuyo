#ifndef _BOB_TOOLBOX_QUADRANT_H_
#define _BOB_TOOLBOX_QUADRANT_H_

namespace bob
{

	//! Represents a Quadrant (can accept values from 1 to 4)
	class Quadrant	
	{

		public:

			Quadrant(int val = 1) : 
			value(normalize(val))
			{}

			operator int()
			{
				return value;
			}

			Quadrant& rotate(int distance)
			{
				value = normalize(value + distance);
				return *this;
			}

		private:

			int normalize(int unbounded)
			{
				return (1 + ((unbounded - 1) % 4));
			}

			int value;

	};

}

#endif

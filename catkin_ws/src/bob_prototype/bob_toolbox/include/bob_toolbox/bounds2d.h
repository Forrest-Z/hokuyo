#ifndef _BOB_TOOLBOX_BOUNDS2D_H_
#define _BOB_TOOLBOX_BOUNDS2D_H_

namespace bob
{

	//! Represents a bounds (or range) in 2D space

	//! The class is designed so that you iteratively add points
	//! and the bounds expands to contain them. Thus, you can
	//! calculate the rectangle that contains a set of points.
	template <typename DataType>
		class Bounds2D
		{

			public:

				//! Create a bounds centered at a particular point
				Bounds2D(GenericPoint<DataType> initial) :
					minX(initial.x),
					maxX(initial.x),
					minY(initial.y),
					maxY(initial.y)
			{}

				DataType minX;
				DataType maxX;

				DataType minY;
				DataType maxY;	

				
				//! Expand the bounds so that it contains the given point
				void expandToCover(GenericPoint<DataType> point)
				{
					minX = std::min(minX, point.x);
					maxX = std::max(maxX, point.x);

					minY = std::min(minY, point.y);
					maxY = std::max(maxY, point.y);
				}

				//! Pad the range in all bounds 
				void pad(DataType amount)
				{
					minX -= amount;
					maxX += amount;

					minY -= amount;
					maxY += amount;
				}

		};

}

#endif

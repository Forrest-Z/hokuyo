#ifndef _BOB_TOOLBOX_DIMENSION2D_H_
#define _BOB_TOOLBOX_DIMENSION2D_H_

namespace bob
{

	//! Represents a 2D width x height dimension
	template <typename DataType>
		struct Dimension2D
		{
			
			//! Simple constructor
			Dimension2D(DataType width=0, DataType height=0) : 
				width(width),
				height(height) 
			{}

			DataType width;
			DataType height;

			inline bool operator==(const Dimension2D<DataType>& other) const
			{
				return (width == other.width) &&
					(height == other.height);
			}

		};

}

#endif

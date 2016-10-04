#ifndef _BOB_TOOLBOX_ARRAY2D_H_
#define _BOB_TOOLBOX_ARRAY2D_H_

#include <assert.h>
#include <bob_toolbox/int_point.h>
#include <bob_toolbox/dimension2d.h>

#include <vector>
#include <algorithm>

namespace bob 
{

	//! A generic 2D array class 
	template <class DataType> 
	class Array2D
	{

		public:

			//! Create empty Array2D
			Array2D() 
			{}

			//! Construct Array2D with specific size
			Array2D(Dimension2D<int> size, DataType defaultValue = DataType());


			typedef typename std::vector<DataType>::reference reference;
			typedef typename std::vector<DataType>::const_reference const_reference;
			typedef IntPoint Point;

			//! Functions used to obtain a reference to a particular cell in the array
			const_reference operator[](const Point& p) const;
			reference operator[](const Point& p);

			//! Detect if a cell is in the array
			bool isInside(const Point& p) const;

			//! Get Array width 
			int getWidth() const 
			{
				return size.width;
			}

			//! Get array height
			int getHeight() const 
			{
				return size.height;
			}

			void fill(const DataType& value)
			{
				std::fill(m_cells.begin(), m_cells.end(), value);
			}

		private:

			//! Calculates the index to look up the value in the container
			int getIndex(Point point) const
			{
				return size.width * point.y + point.x;
			}

			std::vector<DataType> m_cells;
			Dimension2D<int> size;

	};


	template <class DataType>
		Array2D<DataType>::Array2D(Dimension2D<int> size, DataType defaultValue) : 
		size(size),
		m_cells(size.width * size.height, defaultValue)
		{}

	template <class DataType>
		bool Array2D<DataType>::isInside(const Array2D::Point& point) const
		{
			return (point.x >= 0 && point.y >= 0) && 
				(point.x < size.width && point.y < size.height); 
		}

	template <class DataType>
		typename Array2D<DataType>::const_reference Array2D<DataType>::operator[](const Array2D::Point& point) const
		{
			return m_cells[getIndex(point)];
		}


	template <class DataType>
		typename Array2D<DataType>::reference Array2D<DataType>::operator[](const Array2D::Point& point)
		{
			return m_cells[getIndex(point)];
		}

}

#endif


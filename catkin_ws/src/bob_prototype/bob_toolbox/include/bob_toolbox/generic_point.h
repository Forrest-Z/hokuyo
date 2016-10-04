#ifndef _BOB_TOOLBOX_GENERIC_POINT_H_
#define _BOB_TOOLBOX_GENERIC_POINT_H_

#include <cmath>

namespace bob
{
	
	template <typename ValueType>
		struct GenericVector;


	template <typename ValueType>
		struct GenericPoint
		{

			GenericPoint(ValueType x=0, ValueType y=0) :
				x(x),
				y(y)
			{}

			ValueType x;
			ValueType y;

			template <typename OtherType>
			explicit GenericPoint(const GenericPoint<OtherType>& other) :
			x((ValueType)other.x),
			y((ValueType)other.y)
			{}

			operator GenericVector<ValueType>() const
			{
				return GenericVector<ValueType>(x, y);
			}

			GenericVector<ValueType> operator-(const GenericPoint& b) const
			{
				return GenericVector<ValueType>(x - b.x, y - b.y);
			}

			bool operator!=(const GenericPoint& b) const
			{
				return !(x == b.x && y == b.y);
			}

			bool operator==(const GenericPoint& b) const
			{
				return (x == b.x && y == b.y);
			}


		};


	template <typename ValueType>
		struct GenericVector
		{
			
			GenericVector(ValueType x=0, ValueType y=0) :
				x(x),
				y(y)
			{}

			ValueType x;
			ValueType y;

			float getLength()
			{
				return hypot(x, y);
			}

			float getAngle()
			{
				return atan2(y, x);
			}

			operator GenericPoint<ValueType>() const
			{
				return GenericPoint<ValueType>(x, y);
			}

			GenericVector operator-(const GenericVector& b) const
			{
				return GenericVector(x - b.x, y - b.y);
			}

			GenericVector operator+(const GenericVector& b) const
			{
				return GenericVector(x + b.x, y + b.y);
			}

			GenericVector operator/(const float& b) const
			{
				return GenericVector(x / b, y / b);
			}

			bool operator!=(const GenericVector& b) const
			{
				return !(x == b.x && y == b.y);
			}

			bool operator==(const GenericVector& b) const
			{
				return (x == b.x && y == b.y);
			}
		};

	template <typename ValueType>
		GenericVector<ValueType> unitVector(float angle)
		{
			return GenericVector<ValueType>(cos(angle), sin(angle));
		}

	template <typename ValueType>
		GenericPoint<ValueType> operator+(const GenericPoint<ValueType>& point, const GenericVector<ValueType>& vector)
		{
			return GenericPoint<ValueType>(point.x + vector.x, point.y + vector.y);
		}

	template <typename ValueType>
		GenericPoint<ValueType> operator-(const GenericPoint<ValueType>& point, const GenericVector<ValueType>& vector)
		{
			return GenericPoint<ValueType>(point.x - vector.x, point.y - vector.y);
		}


	template <typename ValueType>
		GenericVector<ValueType> operator*(const GenericVector<ValueType>& vector, float factor)
		{
			return GenericVector<ValueType>(factor * vector.x, factor * vector.y);
		}

		//! The following operators don't really do anything. They just delegate to the others, 
		//! so we can perform operations either way (x * y or y * x)
		
	template <typename ValueType>
		GenericPoint<ValueType> operator+(const GenericVector<ValueType>& vector, const GenericPoint<ValueType>& point)
		{
			return operator+(point, vector);
		}

	template <typename ValueType>
		GenericPoint<ValueType> operator-(const GenericVector<ValueType>& vector, const GenericPoint<ValueType>& point)
		{
			return operator-(point, vector);
		}

	template <typename ValueType>
		GenericVector<ValueType> operator*(float factor, const GenericVector<ValueType>& vector)
		{
			return operator*(vector, factor);
		}
}

#endif

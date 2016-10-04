#ifndef _BOB_LIDAR_LIMITED_LIDAR_HANDLE_H_
#define _BOB_LIDAR_LIMITED_LIDAR_HANDLE_H_

#include <bob_ros_implementations/ros_lidar_handle.h>

#include <bob_toolbox/circular_direction.h>
#include <bob_toolbox/angular_range.h>

namespace bob
{

	//! A lidar which has been intentionally handicapped in some way	
	class ROSLimitedLidarHandle : public ROSLidarHandle 
	{

		public:

			ROSLimitedLidarHandle(const ITransformHandle& transformHandle, SimpleAngularRange limit) :
				ROSLidarHandle(transformHandle),
				limit(limit)
		{}

			virtual LidarScan getLidarData() const;

			virtual LidarScan getLidarData(const SimpleAngularRange& angularRange, CircularDirection direction = CounterClockwise) const;

		private:

			SimpleAngularRange limit;

	};

}

#endif

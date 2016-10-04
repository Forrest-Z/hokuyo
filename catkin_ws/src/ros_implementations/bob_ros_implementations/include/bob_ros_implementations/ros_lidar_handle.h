#ifndef _BOB_LIDAR_LIDAR_HANDLE_H_
#define _BOB_LIDAR_LIDAR_HANDLE_H_

#include <bob_sensor/iscan_sensor_handle.h>

#include <bob_ros_implementations/ros_lidar_processor.h>
#include <bob_ros_implementations/ros_lidar_listener.h>
#include <bob_ros_implementations/lidar_util.h>

#include <bob_lidar/lidar_configuration.h>

#include <bob_toolbox/circular_direction.h>

namespace bob
{
	
	//! \brief Provides access to most recent lidar data from sensor.
	//! The data output format is designed to be easy to use in comparison to the raw output
	//! obtained from Lidar sensors.
	class ROSLidarHandle : public IScanSensorHandle
	{

		public:

			//! \brief Construct a ROSLidarHandle
			//! \param zeroAngle The yaw angle of the robot, the angle considered to be 0
			//! \param inverted Whether or not the lidar placement is inverted
			ROSLidarHandle(float zeroAngle = 0, bool inverted = false) :
				lidarProcessor(determineLidarConfiguration(lidarListener, zeroAngle, inverted))
		{}

			//! \brief Construct a ROSLidarHandle using information obtained from a TransformHandle
			ROSLidarHandle(const ITransformHandle& transformHandle) :
				lidarProcessor(determineLidarConfiguration(transformHandle, lidarListener))
		{}

			//! \brief Get the full range of lidar data available
			virtual LidarScan getLidarData() const;

			//! \brief Obtain only a specific lidar range of data 
			//! \param The range of data to obtain
			//! \param The direction that the range will be interpreted
			virtual LidarScan getLidarData(const SimpleAngularRange& angularRange, CircularDirection direction = CounterClockwise) const;

		private:

			//! Subscribes to ROS topic and caches raw data
			ROSLidarListener lidarListener;

			//! Used to convert the data from raw data into LidarScan format
			LidarProcessor lidarProcessor;

	};

}

#endif

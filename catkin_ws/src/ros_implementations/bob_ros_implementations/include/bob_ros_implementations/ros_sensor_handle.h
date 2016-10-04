#ifndef _BOB_ROS_IMPLEMENTATIONS_ROS_SENSOR_HANDLE_H_
#define _BOB_ROS_IMPLEMENTATIONS_ROS_SENSOR_HANDLE_H_

#include <bob_sensor/isensor_handle.h>

#include <bob_ros_implementations/ros_lidar_handle.h>
#include <bob_lidar/lidar_proximity_sensor.h>

#include <bob_ros_implementations/ros_transform_handle.h>
#include <bob_ros_implementations/ros_transform_notifier.h>
#include <bob_ros_implementations/ros_odometry_handle.h>

#include <bob_ros_implementations/ros_bumper_handle.h>
#include <bob_ros_implementations/ros_transform_handle.h>
#include <bob_ros_implementations/ros_lidar_handle.h>

namespace bob
{

	class ROSSensorHandle : public ISensorHandle
	{
		public:
	
			ROSSensorHandle() :
			lidarHandle(transformHandle),
			lidarProximitySensor(lidarHandle)
			{}

			virtual const ITransformHandle& getTransformHandle() const
			{
				return transformHandle;
			}

			virtual const IProximitySensor& getProximitySensor() const
			{
				return lidarProximitySensor;
			}

			virtual const IOdometryHandle& getOdometryHandle() const
			{
				return odometryHandle;
			}

			virtual const IBumperHandle& getBumperHandle() const
			{
				return bumperHandle;
			}

			virtual IBumperHandle& getBumperHandle()
			{
				return bumperHandle;
			}

			virtual const IScanSensorHandle& getScanSensorHandle() const
			{
				return lidarHandle;
			}

			virtual ITransformNotifier& getTransformNotifier()
			{
				return transformNotifier;
			}

		private:

			ROSTransformNotifier transformNotifier;

			ROSTransformHandle transformHandle;

			ROSLidarHandle lidarHandle;

			LidarProximitySensor lidarProximitySensor;	

			ROSOdometryHandle odometryHandle;

			ROSBumperHandle bumperHandle;

			
	};

}

#endif

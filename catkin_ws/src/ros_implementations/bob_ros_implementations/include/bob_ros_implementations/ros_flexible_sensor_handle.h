#ifndef _BOB_ROS_IMPLEMENTATIONS_ROS_FLEXIBLE_SENSOR_HANDLE_H_
#define _BOB_ROS_IMPLEMENTATIONS_ROS_FLEXIBLE_SENSOR_HANDLE_H_

#include <memory>

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

	//! \brief A ROS implementation of ISensorHandle which is designed to maximize flexibility.
	class ROSFlexibleSensorHandle : public ISensorHandle
	{
		public:
	
			ROSFlexibleSensorHandle() :
			scanSensorHandle(new ROSLidarHandle(transformHandle)),
			proximitySensor(new LidarProximitySensor(*scanSensorHandle))
			{}

			void resetScanSensor(std::unique_ptr<IScanSensorHandle> newScanSensor)
			{
				scanSensorHandle.swap(newScanSensor);
				proximitySensor.reset(new LidarProximitySensor(*scanSensorHandle));
			}

			virtual const ITransformHandle& getTransformHandle() const
			{
				return transformHandle;
			}

			virtual const IProximitySensor& getProximitySensor() const
			{
				return *proximitySensor;
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
				return *scanSensorHandle;
			}

			virtual ITransformNotifier& getTransformNotifier()
			{
				return transformNotifier;
			}

		private:

			ROSTransformNotifier transformNotifier;

			ROSTransformHandle transformHandle;

			std::unique_ptr<IScanSensorHandle> scanSensorHandle;

			std::unique_ptr<IProximitySensor> proximitySensor;	

			ROSOdometryHandle odometryHandle;

			ROSBumperHandle bumperHandle;

			
	};

}

#endif

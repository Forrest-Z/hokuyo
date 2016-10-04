#ifndef _BOB_FREERTOS_IMPLEMENTATIONS_FREERTOS_SENSOR_HANDLE_H_
#define _BOB_FREERTOS_IMPLEMENTATIONS_FREERTOS_SENSOR_HANDLE_H_

#include <bob_sensor/isensor_handle.h>

#include <bob_freertos_implementations/freertos_transform_handle.h>
#include <bob_freertos_implementations/freertos_odometry_handle.h>
#include <bob_freertos_implementations/freertos_bumper_handle.h>
#include <bob_freertos_implementations/freertos_scan_sensor_handle.h>

#include <bob_lidar/lidar_proximity_sensor.h>

namespace bob
{

	class ITransformHandle;
	class IProximitySensor;
	class IOdometryHandle;
	class IBumperHandle;
	class IScanSensorHandle;
	class ITransformNotifier;

	//! \brief FreeRTOS implementation of ISensorHandle
	class FreeRTOSSensorHandle : public ISensorHandle
	{

		public:
		
			FreeRTOSSensorHandle() : 
			proximitySensor(scanSensorHandle)
			{}

			virtual const ITransformHandle& getTransformHandle() const
			{
				return transformHandle;
			}

			virtual const IProximitySensor& getProximitySensor() const
			{
				return proximitySensor;
			}

			virtual const IOdometryHandle& getOdometryHandle() const
			{
				return odometryHandle;
			}

			virtual const IBumperHandle& getBumperHandle() const
			{
				return bumperHandle;
			}

			virtual const IScanSensorHandle& getScanSensorHandle() const
			{
				return scanSensorHandle;
			}

			virtual IBumperHandle& getBumperHandle()
			{
				return bumperHandle;
			}

			virtual ITransformNotifier& getTransformNotifier()
			{
				return transformHandle;
			}

		private:

			FreeRTOSTransformHandle transformHandle;
			
			FreeRTOSBumperHandle bumperHandle;
		
			FreeRTOSScanSensorHandle scanSensorHandle;

			FreeRTOSOdometryHandle odometryHandle;

			LidarProximitySensor proximitySensor;

	};

}

#endif

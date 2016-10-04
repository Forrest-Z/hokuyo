#ifndef _BOB_FREERTOS_IMPLEMENTATIONS_FREERTOS_SCAN_SENSOR_HANDLE_H_
#define _BOB_FREERTOS_IMPLEMENTATIONS_FREERTOS_SCAN_SENSOR_HANDLE_H_

#include <bob_toolbox/circular_direction.h>

#include <bob_sensor/iscan_sensor_handle.h>

#include <bob_sensor/lidar_scan.h>

namespace bob
{

	class LidarScan;
	class SimpleAngularRange;

	class FreeRTOSScanSensorHandle : public IScanSensorHandle
	{

		public:

			virtual LidarScan getLidarData() const
			{
				return LidarScan();
			}

			virtual LidarScan getLidarData(const SimpleAngularRange& angularRange, CircularDirection direction = CounterClockwise) const
			{
				return LidarScan();
			}

	};

}

#endif

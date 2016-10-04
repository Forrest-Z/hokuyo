#ifndef _BOB_SENSOR_ISCAN_SENSOR_HANDLE_H_
#define _BOB_SENSOR_ISCAN_SENSOR_HANDLE_H_

#include <bob_toolbox/circular_direction.h>

namespace bob
{

	class LidarScan;
	class SimpleAngularRange;

	//! \brief Interface for a range sensor (LIDAR, or similar). This provides a general interface that can be implemented in order
	//! to test with different types of sensors and lidar.
	class IScanSensorHandle
	{

		public:

			//! \brief Get all of the scan data available
			//! \return The most recent scan information
			virtual LidarScan getLidarData() const = 0;

			//! \brief Get a subsection of the scan data available
			//! \param angularRange The range of angles that are included in the scan
			//! \param direction Defines the ordering of the output scan
			//! \return A subset of the most recent scan information
			virtual LidarScan getLidarData(const SimpleAngularRange& angularRange, CircularDirection direction = CounterClockwise) const = 0;

			virtual ~IScanSensorHandle() 
			{}

	};

}

#endif

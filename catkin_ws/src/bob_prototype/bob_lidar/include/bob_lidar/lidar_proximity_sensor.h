#ifndef _BOB_LIDAR_LIDAR_PROXIMITY_SENSOR_H_
#define _BOB_LIDAR_LIDAR_PROXIMITY_SENSOR_H_

#include <bob_sensor/iproximity_sensor.h>
#include <bob_sensor/iscan_sensor_handle.h>

#include <memory>

namespace bob
{

	class LidarProximitySensor : public IProximitySensor
	{

		public:

			LidarProximitySensor(const IScanSensorHandle& scanSensorHandle) : 
			scanSensorHandle(scanSensorHandle)
			{}

			virtual float distanceFromRobot(float angle) const;

			virtual LidarBeam shortestBeam(const SimpleAngularRange& range) const;

			virtual LidarBeam shortestBeam() const;

			virtual bool firstBeamShorterThan(const SimpleAngularRange& angularRange, const CircularDirection& direction, float minDist, LidarBeam& beam) const;

			virtual AngularRange getPassableRange(float travelDistance, float minMargin) const;

		private:

			LidarBeam shortestBeamInScan(const LidarScan& scan) const;

			const IScanSensorHandle& scanSensorHandle;

	};

}

#endif

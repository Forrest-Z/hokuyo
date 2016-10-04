#include <bob_lidar/lidar_proximity_sensor.h>

#include <limits>

#include <bob_config/config.h>

#include <bob_lidar/util_functions.h>
#include <bob_sensor/lidar_scan.h>

#include <bob_toolbox/circular_direction.h>
#include <bob_toolbox/angular_range.h>

namespace bob
{

	float LidarProximitySensor::distanceFromRobot(float angle) const
	{
		// Get all the beams which are positive when projected onto the angle.
		// These are all the beams from -pi/2 to pi/2 around the angle.
		const float HALF_PI = M_PI / 2;
		SimpleAngularRange range(angle - HALF_PI, angle + HALF_PI);
		LidarScan rangeData = scanSensorHandle.getLidarData(range);

 		float distance = std::numeric_limits<float>::infinity();

		// Loop through front angle of robot, find the shortest front distance point.
		for(LidarScan::iterator it = rangeData.begin(); it != rangeData.end(); ++it)
                {
			float angularDiff = it->angle - angle;
			float newDistance = distanceForBeam(angularDiff, it->range, Config::ROBOT_RADIUS);

			if(newDistance < distance)
			{
				distance = newDistance;
			}
		}
		return distance;
	}

	LidarBeam LidarProximitySensor::shortestBeam(const SimpleAngularRange& range) const
	{
		LidarScan rangeData = scanSensorHandle.getLidarData(range);
		return shortestBeamInScan(rangeData);
	}

	LidarBeam LidarProximitySensor::shortestBeam() const
	{
		LidarScan rangeData = scanSensorHandle.getLidarData();
		return shortestBeamInScan(rangeData);
	}

	LidarBeam LidarProximitySensor::shortestBeamInScan(const LidarScan& scan) const
	{
		float shortestValue = std::numeric_limits<float>::max();
		LidarBeam result;
		for(auto beamItr = scan.begin(); beamItr != scan.end(); ++beamItr)
		{
			if(!isnan(beamItr->range) && beamItr->range < shortestValue)
			{
				shortestValue = beamItr->range;
				result = *beamItr;
			}
		}	

		return result;
	}

	bool LidarProximitySensor::firstBeamShorterThan(const SimpleAngularRange& angularRange, const CircularDirection& direction, float minDist, LidarBeam& beam) const
	{
		LidarScan scan = scanSensorHandle.getLidarData(angularRange, direction);
		for (LidarScan::iterator itr = scan.begin(); itr != scan.end(); ++itr)
		{
			if (!isnan(itr->range) && (itr->range - Config::ROBOT_RADIUS) < minDist)
			{
				beam = *itr;
				return true;	
			}
		}
		return false;
	}

	AngularRange LidarProximitySensor::getPassableRange(float travelDistance, float minMargin) const
	{
		AngularRange unSafeRange;
		float robotRadius = Config::ROBOT_RADIUS;

		LidarScan rangeData = scanSensorHandle.getLidarData();
		for(LidarScan::iterator itr = rangeData.begin(); itr != rangeData.end(); ++itr)
		{	
			// Ignore far points and nan range readings
			if(itr->range > robotRadius + travelDistance || isnan(itr->range))
				continue;
 
			if(itr->range <= robotRadius + minMargin)
			{
				// If the range reading is smaller than robot radius + desired distance away from obstacle,
				// turn at least to make sure the point is at the side of robot.	
				SimpleAngularRange range(itr->angle - M_PI / 2, itr->angle + M_PI / 2);
				unSafeRange.add(range);			
			}
			else
			{
				// Calculate the angle between desired anglular range boundary and current heading
				float angleToTurnAway = asin((robotRadius + minMargin) / itr->range);
				SimpleAngularRange range(itr->angle - angleToTurnAway, itr->angle + angleToTurnAway);
				unSafeRange.add(range);
			}
		}
		
		// Invert to get the safe range;
		return unSafeRange.invert();
	}

}


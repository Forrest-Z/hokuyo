#ifndef _BOB_LIDAR_LIDAR_PROCESSOR_H_
#define _BOB_LIDAR_LIDAR_PROCESSOR_H_

#include <vector>
#include <sensor_msgs/LaserScan.h>

#include <bob_sensor/itransform_handle.h>
#include <bob_toolbox/circular_direction.h>

#include <bob_toolbox/circular_direction.h>

#include <bob_sensor/lidar_scan.h>
#include <bob_lidar/lidar_configuration.h>

namespace bob
{

	class SimpleAngularRange;

	//! This class is used to process Lidar data so that it is easier to use.
	//! It is also intended to standardize the lidar data from different sources.
	//! The data is converted into the robot frame, where 0rad refers to the angle in front of the robot.
	//! It is also possible to choose an angular bounds when processing the scans.
	class LidarProcessor
	{

		public:


			LidarProcessor(LidarConfiguration lidarConfiguration) : 
				lidarConfiguration(lidarConfiguration)
		{}

			//! Returns the LidarScan for a given range, processed in a given direction 
			//
			//! Input:
			//! msg - The lidar message from which to extract the data
			//! startAngle - One of the bounds of the range
			//! endAngle - The other bounds of the range
			//! direction - The direction to process the data
			LidarScan process(const sensor_msgs::LaserScan& msg, float startAngle, float endAngle, CircularDirection direction = CounterClockwise) const;

			LidarScan process(const sensor_msgs::LaserScan& msg, const SimpleAngularRange& angularRange, CircularDirection direction = CounterClockwise) const;

			//! Similar to the other process method, except that it gives all of the lidar data in the direction it was originally obtained
			//
			//! Input:
			//! msg - The lidar message from which to extract the data
			LidarScan process(const sensor_msgs::LaserScan& msg) const;

		private:

			//! Gives the beam angle based on the index of the beam in the scan
			float angleAtIndex(int index) const;

			const LidarConfiguration lidarConfiguration;

			//! Generates a LidarScanSegment from a distance vector. This will always build the scan from startIndex -> endIndex
			//! Without going outside of the vector bounds. If startIndex > endIndex, it will go backwards between them to
			//! generate the scan segment.
			//
			//! Input:
			//! distances - The lidar beam distance data
			//! startIndex - The index of the first element in the LidarScanSegment
			//! endIndex - The index of the last element in the LidarScanSegment
			LidarScanSegment segmentBetweenIndicies(const std::vector<float>& distances, int startIndex, int endIndex) const;

			//! Generates the LidarScanSegments using the distances and the desired start and end indices
			//
			//! Input:
			//! distances - The distances of the lidar beams
			//! startIndex - The first element in the first segment
			//! endIndex - The last element in the last segment
			//! reverse - If true, we are going in the reverse direction in the vector (but still from startIndex -> endIndex)
			std::vector<LidarScanSegment> extractSegments(const std::vector<float>& distances, int startIndex, int endIndex, bool reverse) const;


	};

}

#endif

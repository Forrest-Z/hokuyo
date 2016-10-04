#include <bob_ros_implementations/ros_lidar_processor.h>

#include <bob_sensor/itransform_handle.h>

#include <bob_toolbox/geometry.h>
#include <bob_toolbox/angles.h>

#include <algorithm>
#include <bob_toolbox/angular_range.h>

namespace bob
{

	std::vector<LidarScanSegment> LidarProcessor::extractSegments(const std::vector<float>& distances, int startIndex, int endIndex, bool reverse) const
	{
		std::vector<LidarScanSegment> result;

		if ((startIndex <= endIndex) && reverse)
		{
			// Need to wrap around from the start to the end
			result.push_back(segmentBetweenIndicies(distances, startIndex, 0));
			result.push_back(segmentBetweenIndicies(distances, distances.size() - 1, endIndex));
		}
		else if ((startIndex >= endIndex) && !reverse)
		{
			// Need to wrap around from the end to the start
			result.push_back(segmentBetweenIndicies(distances, startIndex, distances.size() - 1));
			result.push_back(segmentBetweenIndicies(distances, 0, endIndex));
		}
		else
		{
			// Simple case
			result.push_back(segmentBetweenIndicies(distances, startIndex, endIndex));
		}
		return result;
	}

	LidarScanSegment LidarProcessor::segmentBetweenIndicies(const std::vector<float>& distances, int startIndex, int endIndex) const
	{
		// Determine which way we are going along the vector
		bool reverse = (startIndex > endIndex);
		
		float newOffset = angleAtIndex(startIndex);

		// To go in reverse we swap the start and end, then reverse the resulting vector
		if (reverse)
			std::swap(startIndex, endIndex);

		std::vector<float> extractedDistances(distances.begin() + startIndex, distances.begin() + endIndex + 1);

		if (reverse)
			std::reverse(extractedDistances.begin(), extractedDistances.end());

		// If we are going in reverse then the increment needs to be negated
		float effectiveIncrement = reverse ? -lidarConfiguration.robotAngleIncrement : lidarConfiguration.robotAngleIncrement;

		return LidarScanSegment(newOffset, effectiveIncrement, extractedDistances); 
	}

	float LidarProcessor::angleAtIndex(int index) const
	{
		return lidarConfiguration.robotAngleMin + index * lidarConfiguration.robotAngleIncrement;
	}

	LidarScan LidarProcessor::process(const sensor_msgs::LaserScan& msg) const
	{
		LidarScanSegment onlySegment(lidarConfiguration.robotAngleMin, lidarConfiguration.robotAngleIncrement, std::vector<float>(msg.ranges.begin(), msg.ranges.end()));
		return LidarScan(onlySegment);
	}

	LidarScan LidarProcessor::process(const sensor_msgs::LaserScan& msg, const SimpleAngularRange& angularRange, CircularDirection direction) const
	{
		return process(msg, angularRange.lower, angularRange.upper, direction);
	}

	LidarScan LidarProcessor::process(const sensor_msgs::LaserScan& msg, float startAngle, float endAngle, CircularDirection direction) const
	{
		// The offset of the start and end angle from the min angle
		float startOffset;
		float endOffset;

		if (lidarConfiguration.robotAngleIncrement > 0)
		{
			startOffset = startAngle - lidarConfiguration.robotAngleMin;
			endOffset = endAngle - lidarConfiguration.robotAngleMin;
		}
		else
		{
			startOffset = lidarConfiguration.robotAngleMin - startAngle;
			endOffset = lidarConfiguration.robotAngleMin - endAngle;
		}

		// This can cause problems if someone gives a range [0, 2pi]
		startOffset = normalizeAnglePos(startOffset);
		endOffset = normalizeAnglePos(endOffset);

		// Conert these angles to indicies
		int startIndex = startOffset / fabs(lidarConfiguration.robotAngleIncrement);
		int endIndex = endOffset / fabs(lidarConfiguration.robotAngleIncrement);

		// Make sure we don't get out of bounds errors
		startIndex = std::min((int)(msg.ranges.size() - 1), startIndex);
		endIndex = std::min((int)(msg.ranges.size() - 1), endIndex);

		// Determine if we must go backwards in the lidar message
		bool reverse;
		if ((lidarConfiguration.robotAngleIncrement > 0 && direction == Clockwise) ||
				(lidarConfiguration.robotAngleIncrement < 0 && direction == CounterClockwise))
		{
			reverse = true;
		}
		else
		{
			reverse = false;
		}

		
		std::vector<LidarScanSegment> segments = extractSegments(msg.ranges, startIndex, endIndex, reverse);

		return LidarScan(segments);	
	}
}


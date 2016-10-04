#ifndef _BOB_SLAM_SCAN_SCORING_H_
#define _BOB_SLAM_SCAN_SCORING_H_

#include <bob_grid_map/iprobability_map.h>

#include <bob_toolbox/int_point.h>
#include <bob_toolbox/pose2d.h>
#include <bob_toolbox/eight_direction.h>

#include <bob_sensor/lidar_scan.h>

#include <vector>

namespace bob
{

	struct ScanScore
	{
		ScanScore() : matchRatio(0), unknownRatio(0) {}

		float matchRatio;
		float unknownRatio;
	};

	ScanScore scoreScanNarrowFOV(const IProbabilityMap& map, const Pose2D& lidarPose, const LidarScan& readings);

	ScanScore scoreScan(const IProbabilityMap& map, const Pose2D& lidarPose, const LidarScan& readings);

	bool probabilitSmallerThan(const std::vector<IntPoint> points, float probablity, const IProbabilityMap& map);

	bool pointsMatchMap(IntPoint hitPoint, IntPoint freePoint, const IProbabilityMap& map);

	bool shiftedPointsMatch(IntPoint unshiftedHitPoint, IntPoint unshiftedFreePoint, std::vector<EightDirection> directions, const IProbabilityMap& map);

}

#endif

#ifndef _BOB_SLAM_MAP_FILLER_H_
#define _BOB_SLAM_MAP_FILLER_H_

#include <bob_grid_map/iprobability_map.h>

#include <bob_toolbox/pose2d.h>
#include <bob_sensor/lidar_scan.h>

#include <vector>


namespace bob
{

	class MapFiller
	{

		public:

			MapFiller(float maxRange, float maxURange) :
				maxRange(maxRange),
				maxURange(maxURange)
		{}

			void updateMap(IProbabilityMap& map, const Pose2D& lidarPose, const LidarScan& readings, bool fillFree);

			void resizeMap(IProbabilityMap& map, const Pose2D& lidarPose, const LidarScan& readings);

			void registerScan(IProbabilityMap& map, const Pose2D& lidarPose, const LidarScan&, bool fillFree);

		private:


			float maxRange;
			float maxURange;


	};

}

#endif

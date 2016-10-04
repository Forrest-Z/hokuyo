#ifndef _BOB_SLAM_SLAM_PROCESSOR_NARROW_FOV_H_
#define _BOB_SLAM_SLAM_PROCESSOR_NARROW_FOV_H_

#include <bob_sensor/lidar_scan.h>

#include <bob_grid_map/decay_probability_cell.h>
#include <bob_grid_map/concrete_map.h>
#include <bob_grid_map/map_sender.h>

#include <bob_slam/icp_scan_matcher.h>
#include <bob_slam/map_filler.h>

#include <bob_system/system_utilities.h>

#include <bob_sensor/itransform_notifier.h>

namespace bob
{
	//! \brief This class is responsible for updating robot pose estimation,
	//! publishing transform and updating map.
	class SlamProcessorNarrowFOV
	{

		public:
			//! \brief Constructor
			//! \param mapSender Used to access current costmap 
			SlamProcessorNarrowFOV(MapSender& mapSender, ITransformNotifier& transformNotifier) : 
				transformPublisher(transformNotifier),
				poseEstimate(),
				lastOdomPose(),
				scanMatcher(),
				mapFiller(Config::MAX_SENSOR_RANGE, Config::MAX_SENSOR_RANGE),
				mapSender(mapSender),
				firstOdomReceived(false),
				firstScanProcessed(false),
				mapFillingTimer(systemUtilities->timer())
			{
				// Initialize low resolution map
				double resolution = mapSender.getMap().getLocationMapper().getResolution();
				for (int i = 1; i < 2; ++i)
				{
					lowResMaps.push_back(LowResMapType(resolution * pow(2, i)));
				}	
				mapFillingTimer->startTimer();
			}	

			//! \brief Process LidarScan, update robot pose estimate, map to odom transform 
			//! and map.
			//! \param odomPose Robot pose relative to odom frame
			//! \param reading Most recent LidarScan 
			void addScan(const Pose2D& odomPose, const LidarScan& reading);

		private: 

			typedef ConcreteMap<DecayProbabilityCell> LowResMapType;

			void advancePoseEstimate(const Pose2D& odomPose);

			bool scanMatchPose(const LidarScan& laserReading, IProbabilityMap& map);

			void updateMap(const LidarScan& reading, IProbabilityMap& map);

			void updateTransform(const Pose2D& odomPose);

			Pose2D poseEstimate;

			Pose2D lastOdomPose;

			ICPScanMatcher scanMatcher;

			std::vector<LowResMapType> lowResMaps;

			MapFiller mapFiller;

			MapSender& mapSender;

			ITransformNotifier& transformPublisher;

			bool firstOdomReceived;
			bool firstScanProcessed;

			//! Last time a map was published
			Timer mapFillingTimer;


	};

}

#endif

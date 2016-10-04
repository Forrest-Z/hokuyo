#include <bob_slam/slam_processor_narrow_fov.h>
#include <bob_slam/scan_scoring.h>
#include <bob_slam/visualization.h>

#include <bob_lidar/util_functions.h>
#include <bob_visualization/visualization.h>

#include <bob_toolbox/matrix3.h>
#include <bob_toolbox/logging.h>
#include <bob_toolbox/easy_print.h>
#include <bob_toolbox/matrix_operations.h>

namespace bob
{

	void SlamProcessorNarrowFOV::addScan(const Pose2D& odomPose, const LidarScan& reading)
	{
		// Initialize lastOdomPose
		if (!firstOdomReceived)
		{
			lastOdomPose = odomPose;
			firstOdomReceived = true;
		}
	
		// Update the pose estimate
		advancePoseEstimate(odomPose);

		StrictLock lock(mapSender.getMap().getLock());
		Costmap& costmap = mapSender.getMap().getLockedResource(lock);

		bool allowMapUpdate = false;
		if (firstScanProcessed)
		{
			// If initialized, scan match pose
			allowMapUpdate = scanMatchPose(reading, costmap.getProbabilityMap());
		}

		if (!firstScanProcessed || 
			(allowMapUpdate && mapFillingTimer->timeElapsed() > 3))
		{
			// Update map
			updateMap(reading, costmap.getProbabilityMap());
			mapFillingTimer->startTimer();
			firstScanProcessed = true;
		}

		
		// Update transform from map to odom
		updateTransform(odomPose);
	}
		
	void SlamProcessorNarrowFOV::advancePoseEstimate(const Pose2D& odomPose)
	{
		// Advance poseEstimate based on odom increment
		poseEstimate += (odomPose - lastOdomPose);
		lastOdomPose = odomPose;
		Matrix3 odomCov = Matrix3::zeros();
		odomCov(0, 0) = 0.1;
		odomCov(1, 1) = 0.1;
		odomCov(2, 2) = 0.2;
		visualizer->visualize("odomCov", visualization(poseEstimate, odomCov), greenMarkerStyle());
		LOG_SLAM("odomCov: \n" << odomCov);
	}

	bool SlamProcessorNarrowFOV::scanMatchPose(const LidarScan& laserReading, IProbabilityMap& map)
	{
		// Initialize corrected pose with odom incremented poseEstimate
		Pose2D scanEstPose = poseEstimate;
		Matrix3 covMatrix;

		// Scan matching with low resolution maps
		for (std::vector<LowResMapType>::iterator mapItr = lowResMaps.begin(); mapItr != lowResMaps.end(); ++mapItr)
		{
			scanEstPose = scanMatcher.scanMatchPose(*mapItr, scanEstPose, laserReading, covMatrix);
		}

		// Scan matching with high resolution map
		scanEstPose = scanMatcher.scanMatchPose(map, scanEstPose, laserReading, covMatrix);
		
		LOG_SLAM("covMatrix: \n" << covMatrix);
		
		visualizer->visualize("Hessian", visualization(scanEstPose, covMatrix));

		ScanScore score = scoreScanNarrowFOV(map, scanEstPose, laserReading);

		// Get jump distance between odom predicted pose and scan matching corrected pose
		WorldVector linearJump = (scanEstPose - poseEstimate);
		float jumpDistance = linearJump.getLength();
		
		// Corrected poseEstimate if the scan matching score indicates a high possibility poseEstimate 
		// and jump distance is smaller than minimum allowed jump distance
		// Otherwise, keep the odom predicted pose as poseEstimate
		if (score.matchRatio > Config::MIN_SCAN_MATCH_SCORE && score.unknownRatio < 0.5 && jumpDistance < 0.5)
		{
			// Scan matching succeeded
			poseEstimate = scanEstPose;
			return true;
		}
		// else if (score.unknownRatio >= 0.7)
		// {
		// 	return true;	
		// }
		else
		{
			return false;
		}
	}

	void SlamProcessorNarrowFOV::updateMap(const LidarScan& reading, IProbabilityMap& map)
	{
		// Update low resolution maps
		for (std::vector<LowResMapType>::iterator mapItr = lowResMaps.begin(); mapItr != lowResMaps.end(); ++mapItr)
		{
			mapFiller.updateMap(*mapItr, poseEstimate, reading, true);
		}

		// Update high resolution map
		mapFiller.updateMap(map, poseEstimate, reading, true);

	}

	void SlamProcessorNarrowFOV::updateTransform(const Pose2D& odomPose)
	{
		visualizer->visualize("LocalizedPose", MarkerArrow(poseEstimate));
		visualizer->visualize("RobotShell", MarkerCircle(poseEstimate, Config::ROBOT_RADIUS));

		Pose2D offsetError = poseEstimate * odomPose.generateInversion();

		transformPublisher.updateMapToOdom(offsetError);
	}
}


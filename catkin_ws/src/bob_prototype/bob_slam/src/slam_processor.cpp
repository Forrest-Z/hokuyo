#include <bob_slam/slam_processor.h>
#include <bob_slam/scan_scoring.h>

#include <bob_config/config.h>
#include <bob_visualization/visualization.h>

#include <bob_lidar/util_functions.h>

#include <bob_toolbox/logging.h>
#include <bob_toolbox/matrix3.h>

namespace bob
{

	void SlamProcessor::addScan(const Pose2D& odomPose, const LidarScan& reading)
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

		bool scanMatchSucceeded = false;
		if (firstScanProcessed)
		{
			// If initialized, scan match pose
			scanMatchSucceeded = scanMatchPose(reading, costmap.getProbabilityMap());
		}

		if (!firstScanProcessed || 
			(scanMatchSucceeded && (mapFillingTimer->timeElapsed() > 3)))
		{
			// Update map
			updateMap(reading, costmap.getProbabilityMap());
			mapFillingTimer->startTimer();
			firstScanProcessed = true;
		}
		
		// Update transform from map to odom
		updateTransform(odomPose);
	}
		
	void SlamProcessor::advancePoseEstimate(const Pose2D& odomPose)
	{
		// Advance poseEstimate based on odom increment
		poseEstimate += (odomPose - lastOdomPose);
		lastOdomPose = odomPose;
	}

	bool SlamProcessor::scanMatchPose(const LidarScan& laserReading, IProbabilityMap& map)
	{
		// Initialize corrected pose with odom incremented poseEstimate
		Pose2D tempCorrectedPose = poseEstimate;
		Matrix3 covMatrix;		
		
		// Scan matching with low resolution maps
		for (std::vector<LowResMapType>::iterator mapItr = lowResMaps.begin(); mapItr != lowResMaps.end(); ++mapItr)
		{
			tempCorrectedPose = scanMatcher.scanMatchPose(*mapItr, tempCorrectedPose, laserReading, covMatrix);
		}

		// Scan matching with high resolution map
		tempCorrectedPose = scanMatcher.scanMatchPose(map, tempCorrectedPose, laserReading, covMatrix);

		ScanScore score = scoreScan(map, tempCorrectedPose, laserReading);

		// Get jump distance between odom predicted pose and scan matching corrected pose
		WorldVector linearJump = (tempCorrectedPose - poseEstimate);
		float jumpDistance = linearJump.getLength();
		
		// Corrected poseEstimate if the scan matching score indicates a high possibility poseEstimate 
		// and jump distance is smaller than minimum allowed jump distance
		// Otherwise, keep the odom predicted pose as poseEstimate
		if (score.matchRatio > Config::MIN_SCAN_MATCH_SCORE && score.unknownRatio < 0.7 && jumpDistance < 0.2)
		{
			// Scan matching succeeded
			poseEstimate = tempCorrectedPose;
			return true;
		}
		else
		{
			return false;
		}
	}

	void SlamProcessor::updateMap(const LidarScan& reading, IProbabilityMap& map)
	{
		// Update low resolution maps
		for (std::vector<LowResMapType>::iterator mapItr = lowResMaps.begin(); mapItr != lowResMaps.end(); ++mapItr)
		{
			mapFiller.updateMap(*mapItr, poseEstimate, reading, true);
		}

		// Update high resolution map
		mapFiller.updateMap(map, poseEstimate, reading, true);
	}

	void SlamProcessor::updateTransform(const Pose2D& odomPose)
	{
		visualizer->visualize("LocalizedPose", MarkerArrow(poseEstimate));
		visualizer->visualize("RobotShell", MarkerCircle(poseEstimate, Config::ROBOT_RADIUS));

		Pose2D offsetError = poseEstimate * odomPose.generateInversion();

		transformPublisher.updateMapToOdom(offsetError);
	}
}


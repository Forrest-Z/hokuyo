#include <bob_slam/binary_search_scan_matcher.h>

#include <cstring>
#include <limits>
#include <list>
#include <iostream>
#include <ros/ros.h>

#include <bob_toolbox/grid_algorithms.h>
#include <bob_slam/scan_scoring.h>

#include <bob_config/config.h>

namespace bob 
{

	// Implements a sort of pseudo binary-search. Searches around the robot and halves the search distance each time.
	// Returns the pose found using this binary search as the best scoring pose
	Pose2D BinarySearchScanMatcher::scanMatchPose(const ConcreteMap& map, const Pose2D& init, const LaserReading& readings, const std::vector<float>& laserAngles)
	{
		float bestScore = -1;
		Pose2D currentPose = init;
		float currentScore = scoreScan(map, currentPose, readings, laserAngles);
		float adelta = oldConfig("binary_scan_matcher")("a_step");
		float ldelta = oldConfig("binary_scan_matcher")("l_step");
		unsigned int iterations = 0;
		enum Move { Front, Back, Left, Right, TurnLeft, TurnRight, Done };
		do
		{
			if (bestScore >= currentScore)
			{
				iterations++;
				adelta *= 0.5;
				ldelta *= 0.5;
			}
			bestScore = currentScore;
			Pose2D bestLocalPose = currentPose;
			Pose2D localPose = currentPose;

			// Try a bunch of poses around the localPose
			Move move = Front;
			do 
			{
				localPose = currentPose;
				switch(move)
				{
					case Front:
						localPose.x += ldelta;
						move = Back;
						break;
					case Back:
						localPose.x -= ldelta;
						move = Left;
						break;
					case Left:
						localPose.y -= ldelta;
						move = Right;
						break;
					case Right:
						localPose.y += ldelta;
						move = TurnLeft;
						break;
					case TurnLeft:
						localPose.theta += adelta;
						move = TurnRight;
						break;
					case TurnRight:
						localPose.theta -= adelta;
						move = Done;
						break;
					default:;
				}

				float localScore = scoreScan(map, localPose, readings, laserAngles);

				if (localScore > currentScore)
				{
					currentScore = localScore;
					bestLocalPose = localPose;
				}

			} while(move != Done);

			// Save the best local pose
			currentPose = bestLocalPose;

		} while (currentScore > bestScore || iterations < oldConfig("binary_scan_matcher")("iterations"));

		// Return the pose found
		return currentPose;
	}

};


#include <bob_slam/scan_scoring.h>

#include <bob_grid_map/counting_probability_cell.h>
#include <bob_toolbox/world_point.h>
#include <bob_toolbox/raytrace.h>

#include <bob_toolbox/logging.h>

namespace bob
{

	ScanScore scoreScanNarrowFOV(const IProbabilityMap& map, const Pose2D& lidarPose, const LidarScan& readings) 
	{
		ScanScore score;

		float totalObstacleScore = 0;
		int numUnknownBeams = 0;
		int numExactMatch = 0;
		int numEffectiveBeams = 0;

		
		for (LidarScan::iterator readingItr = readings.begin(); 
				readingItr != readings.end(); 
				++readingItr)
		{
			float beamDistance = readingItr->range;
			float beamAngle = readingItr->angle;

			// Throw out scans due to max usable range condition
			if (beamDistance > Config::MAX_SENSOR_RANGE || isnan(beamDistance)) continue;
			
			numEffectiveBeams++;			

			// End point for this beam
			WorldPoint endPoint = lidarPose + beamDistance * unitVector(lidarPose.theta + beamAngle);

			IntPoint endPointMapLoc = map.worldToMap(endPoint);

			// Free cell next to the end point
			float distanceFreeInFront = sqrt(2)/2;
			WorldVector freeDirection = -distanceFreeInFront * unitVector(lidarPose.theta + beamAngle);
			IntVector freeVector(freeDirection.x / map.getResolution(), freeDirection.y / map.getResolution());
			IntPoint freePointMapLoc = endPointMapLoc + IntVector(freeVector.x, freeVector.y);

			//TODO:from to should be world point
			std::vector<IntPoint> freePoints = raytraceLine<IntPoint>(map.worldToMap(lidarPose), freePointMapLoc);	
			freePoints.erase(freePoints.end()-1);

			float endPointProbability = map.getProbability(endPointMapLoc);
			if (endPointProbability < 0 && probabilitSmallerThan(freePoints, 0.25, map))
			{
				numUnknownBeams++;
			}
			else 
			{
				if (endPointProbability > 0.25 && probabilitSmallerThan(freePoints, 0.25, map))
				{
					numExactMatch++;
				}
			}
			
			/*
			float bestMuVal = -1;
			if (pointsMatchMap(endPointMapLoc, freePointMapLoc, map))
			{
				// Point is an exact match
				bestMuVal = 0;
				numExactMatch++;
				
			}
			else if (shiftedPointsMatch(endPointMapLoc, freePointMapLoc, nondiagonalEightDirections(), map))
			{
				// Point is a match horizontally or vertical
				bestMuVal = map.getResolution();
			}
			else if (shiftedPointsMatch(endPointMapLoc, freePointMapLoc, diagonalEightDirections(), map))
			{
				// Point is a match diagonally
				bestMuVal = sqrt(2) * map.getResolution();
			}
			else
			{
				numUnknownBeams++;				
			}

			if (bestMuVal != -1)
			{
				float sigma = 0.05;
				// Adjust the score and likelihood
				totalObstacleScore += exp((- 1.0/ sigma) * bestMuVal);
				numObstacleBeams++;
			}
			*/
		}
		score.matchRatio = (numEffectiveBeams - numUnknownBeams) == 0 ? 0 : (float)numExactMatch / (float)(numEffectiveBeams - numUnknownBeams);
		score.unknownRatio = (numEffectiveBeams == 0) ? 0 : (float)numUnknownBeams / (float)numEffectiveBeams;
		LOG_SLAM("total effective beam: " << numEffectiveBeams << " unknown beam: " << numUnknownBeams << " known beam: " << numEffectiveBeams - numUnknownBeams << " exact match beam: " << numExactMatch);
		LOG_SLAM("match ratio: " << score.matchRatio << " unknow ratio: " << score.unknownRatio);
		return score;
	}

	// TODO: just for testin g, need to refactor
	bool probabilitSmallerThan(const std::vector<IntPoint> points, float probablity, const IProbabilityMap& map) 
	{
		for (std::vector<IntPoint>::const_iterator itr = points.begin(); itr != points.end(); ++itr)
		{
			if (map.getProbability(*itr) > probablity)
				return false;
		}
		
		return true;
	}


	ScanScore scoreScan(const IProbabilityMap& map, const Pose2D& lidarPose, const LidarScan& readings) 
	{
		ScanScore score;

		float totalObstacleScore = 0;
		int numObstacleBeams = 0;
		int numUnknownBeams = 0;

		
		for (LidarScan::iterator readingItr = readings.begin(); 
				readingItr != readings.end(); 
				++readingItr)
		{
			float beamDistance = readingItr->range;
			float beamAngle = readingItr->angle;

			// Throw out scans due to max usable range condition
			if (beamDistance > Config::MAX_SENSOR_RANGE || isnan(beamDistance)) continue;
			
			// End point for this beam
			WorldPoint endPoint = lidarPose + beamDistance * unitVector(lidarPose.theta + beamAngle);

			IntPoint endPointMapLoc = map.worldToMap(endPoint);

			float distanceFreeInFront = sqrt(2)/2;
			WorldVector freeDirection = -distanceFreeInFront * unitVector(lidarPose.theta + beamAngle);
			IntVector freeVector(freeDirection.x / map.getResolution(), freeDirection.y / map.getResolution());

			IntPoint freePointMapLoc = endPointMapLoc + IntVector(freeVector.x, freeVector.y);

			float bestMuVal = -1;
			if (pointsMatchMap(endPointMapLoc, freePointMapLoc, map))
			{
				// Point is an exact match
				bestMuVal = 0;
			}
			else if (shiftedPointsMatch(endPointMapLoc, freePointMapLoc, nondiagonalEightDirections(), map))
			{
				// Point is a match horizontally or vertical
				bestMuVal = map.getResolution();
			}
			else if (shiftedPointsMatch(endPointMapLoc, freePointMapLoc, diagonalEightDirections(), map))
			{
				// Point is a match diagonally
				bestMuVal = sqrt(2) * map.getResolution();
			}
			else
			{
				numUnknownBeams++;				
			}

			if (bestMuVal != -1)
			{
				float sigma = 0.05;
				// Adjust the score and likelihood
				totalObstacleScore += exp((- 1.0/ sigma) * bestMuVal);
				numObstacleBeams++;
			}
		}
		score.matchRatio = totalObstacleScore / (float)numObstacleBeams;
		score.unknownRatio = (float)numUnknownBeams / (float)readings.size();

		LOG_SLAM("number of matched points: " << numObstacleBeams);
		LOG_SLAM("match ratio: " << score.matchRatio << "unknow ratio: " << score.unknownRatio); 
		return score;
	}

	bool shiftedPointsMatch(IntPoint unshiftedHitPoint, IntPoint unshiftedFreePoint, std::vector<EightDirection> directions, const IProbabilityMap& map)
	{
		for (std::vector<EightDirection>::iterator directionItr = directions.begin(); directionItr != directions.end(); ++directionItr)
		{
			IntPoint testedHitPoint = cellInDirection(unshiftedHitPoint, *directionItr);
			IntPoint testedFreePoint = cellInDirection(unshiftedFreePoint, *directionItr);
			if (pointsMatchMap(testedHitPoint, testedFreePoint, map))
			{
				return true;
			}
		}
		return false;
	}

	bool pointsMatchMap(IntPoint hitPoint, IntPoint freePoint, const IProbabilityMap& map)
	{
		float mapHitProbability = map.getProbability(hitPoint);
		float mapFreeProbability = map.getProbability(freePoint);

		// Arbitrary, should be parameritized
		float obstacleProbability = 0.1;

		return (mapHitProbability > obstacleProbability && 
			mapFreeProbability < obstacleProbability);
	}

}


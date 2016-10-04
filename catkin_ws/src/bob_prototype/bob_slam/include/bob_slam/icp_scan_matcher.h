#ifndef _BOB_SLAM_ICP_SCAN_MATCHER_H_
#define _BOB_SLAM_ICP_SCAN_MATCHER_H_

#include <bob_slam/map_point_cloud.h>
#include <bob_toolbox/pose2d.h>
#include <bob_grid_map/iprobability_map.h>
#include <bob_sensor/lidar_scan.h>

namespace bob 
{

	class Matrix3;

	//! Implements an Iterative Closest Point scan matching algorithm. 

	//! The algorithm is conceptually similar  to "Newton's method." Newton's method solves for the minimum of a
	//! function of a single variable using successive approximations calculated via gradient descent. This scan
	//! matcher, on the other hand, is solving for the minimum of a function of 3 variables. These variables
	//! represent the pose of the robot [x, y, theta]. The function is specificially designed so that it is at
	//! a minimum when the scan is matched with the map.
	//!
	//! The algorithm is commonly known as "hector_slam." You can google that to get some papers describing the
	//! technique in more detail.
	class ICPScanMatcher
	{

		public:

			//! Create a scan matcher object and specify the number of iterations to use
			ICPScanMatcher(int numIterations = 10) : 
			numIterations(numIterations)
			{}

			//! Attempts to determine the likeliest robot pose 

			//! @param map The existing map to compare against
			//! @param beginEstimateWorld A pose estimate to use as the starting value in the scan matching
			//! @param readings The lidar data to use for matching with the map
			Pose2D scanMatchPose(const IProbabilityMap& map, const Pose2D& seedPose, const LidarScan& readings, Matrix3& covMatrix);


		private:

	
			//! Defines a pose in map-coordinates. This is similar to Pose2D except everything is scaled by map resolution.
			//! So, here the position is actually in the map co-ordinates and not world co-ordinates.
			struct MapPose
			{
				GenericPoint<float> position;
				float theta;
				
				MapPose(GenericPoint<float> position, float theta) : 
				position(position),
				theta(theta)
				{}
			};

			//! Stores relevant data calculated from a point, which is used as input for the hessian matrix calculation
			struct ProcessedPointData
			{
				//! The probability of the point in the map, calculated by taking a weighted average of the
				//! probability of the surrounding points.
				float interpolatedProbability;


				GenericVector<float> gradient;
			};

			//! Descends the gradient, giving a better approximation of the pose
			void descendGradient(MapPose& estimate, const MapPointCloud& dataPoints, const IProbabilityMap& map, Matrix3& covMatrix) const;

			//! Calculates the vector pointing in the direction of gradient descent
			MapPose calculateGradientVector(const MapPose& pose, const MapPointCloud& reading, const IProbabilityMap& map, Matrix3& covMatrix) const;
			MapPose calculateGradientVectorUseFreeCells(const MapPose& pose, const MapPointCloud& reading, const IProbabilityMap& map) const;

			//! Determines the relevant data associated with a point
			ProcessedPointData processPoint(const GenericPoint<float>& mapCoords, const IProbabilityMap& map) const;

			//! Transforms a point from the robot frame to the world frame
		
			//! @param position The robot position
			//! @param cosRot The cosine of the robot orientation. The cosine is accepted instead of the raw value to avoid recalculation,
			//! for optimization purposes.
			//! @param sinRot The sine  of the robot orientation
			//! @param point The point to transform into map co-ordinates
			GenericPoint<float> robotToMap(const GenericPoint<float> position, float cosRot, float sinRot, GenericPoint<float> point) const;

			//! Number of successive approximations to generate
			const int numIterations;			

	};

}


#endif

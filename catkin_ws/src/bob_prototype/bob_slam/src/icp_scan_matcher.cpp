#include <bob_slam/icp_scan_matcher.h>

#include <bob_toolbox/matrix3.h>
#include <bob_toolbox/matrix_operations.h>
#include <bob_toolbox/vector3.h>
#include <bob_lidar/util_functions.h>
#include <bob_toolbox/generic_point.h>
#include <bob_toolbox/int_point.h>
#include <bob_toolbox/geometry.h>
#include <bob_toolbox/angles.h>
#include <bob_toolbox/logging.h>

namespace bob 
{
	Pose2D ICPScanMatcher::scanMatchPose(const IProbabilityMap& map, const Pose2D& seedPose, const LidarScan& readings, Matrix3& covMatrix)
	{
		// Get the end point
		MapPointCloud pointCloud(readings, map);
	
		// Point cloud might be empty in rare cases because all values of lidar beam are nan, inf, etc.
		if (pointCloud.isEmpty())
			return seedPose;

		// Scale to map
		MapPose estimateMapCoords(map.worldToMapContinuous(seedPose), seedPose.theta);

		for (int i = 0; i < numIterations; ++i) 
		{
			descendGradient(estimateMapCoords, pointCloud, map, covMatrix);
		}

		// Normalize the angle because there is a small chance it could grow unbounded
		estimateMapCoords.theta = normalizeAngle(estimateMapCoords.theta);

		// Rescale to world and return
		return Pose2D(map.mapToWorldContinuous(estimateMapCoords.position), estimateMapCoords.theta);
	}

	void ICPScanMatcher::descendGradient(MapPose& estimate, const MapPointCloud& dataPoints, const IProbabilityMap& map, Matrix3& covMatrix) const
	{
		MapPose searchVector = calculateGradientVector(estimate, dataPoints, map, covMatrix);

		estimate.position.x += searchVector.position.x;
		estimate.position.y += searchVector.position.y;
		estimate.theta += searchVector.theta;
	}

	ICPScanMatcher::MapPose ICPScanMatcher::calculateGradientVector(const MapPose& pose, const MapPointCloud& reading, const IProbabilityMap& map, Matrix3& covMatrix) const
	{
		float sinRot = sin(pose.theta);
		float cosRot = cos(pose.theta);

		Matrix3 H = Matrix3::zeros();
		Vector3f dTr = Vector3f::zeros();

		for (MapPointCloud::iterator itr = reading.begin(); itr != reading.end(); ++itr) 
		{
			// Convert the end point to map frame
			GenericPoint<float> mapPoint = robotToMap(pose.position, cosRot, sinRot, *itr);
			ProcessedPointData pointData = processPoint(mapPoint, map);

			float freeProbability = 1.0f - pointData.interpolatedProbability;
			float rotationDerivative = ((-sinRot * itr->x - cosRot * itr->y) * pointData.gradient.x + 
					   (cosRot * itr->x - sinRot * itr->y) * pointData.gradient.y);

			dTr(0) += freeProbability * pointData.gradient.x;
			dTr(1) += freeProbability * pointData.gradient.y;
			dTr(2) += freeProbability * rotationDerivative;

			// Calculate the Hessian Diagonals
			H(0, 0) += pow(pointData.gradient.x, 2);
			H(1, 1) += pow(pointData.gradient.y, 2);
			H(2, 2) += pow(rotationDerivative, 2);

			// Calculate the off-diagonals
			H(0, 1) += pointData.gradient.x * pointData.gradient.y;
			H(2, 0) += rotationDerivative * pointData.gradient.x;
			H(2, 1) += rotationDerivative * pointData.gradient.y;
		}

		// Copy the off-diagonals (Hessian is symmetric)
		H(0, 1) = H(1, 0);
		H(0, 2) = H(2, 0);
		H(1, 2) = H(2, 1);

		// Note: H is the covariance Matrix!!
		// This has the potential for future use in Kalmann filter or similar
		//covMatrix = H;

		Vector3f toReturn;
		Matrix3 invH;
		if (inverse(H, invH)) 
		{
			toReturn =  invH * dTr;
		}
		else
		{
			// Hessian is non-inertible for whatever reason
			toReturn = Vector3f::zeros();
		}
		
		covMatrix = invH;
		MapPose realReturn(GenericPoint<float>(toReturn(0), toReturn(1)), toReturn(2));
		return realReturn;
	}


	ICPScanMatcher::MapPose ICPScanMatcher::calculateGradientVectorUseFreeCells(const MapPose& pose, const MapPointCloud& reading, const IProbabilityMap& map) const
	{	
		float sinRot = sin(pose.theta);
		float cosRot = cos(pose.theta);

		Matrix3 H = Matrix3::zeros();
		Vector3f dTr = Vector3f::zeros();

		float mapResolution = map.getResolution();	
		float increment = sqrt(2) * mapResolution;
		
		int i = 0;
		for (MapPointCloud::iterator itr = reading.begin(); itr != reading.end(); ++itr) 
		{
			GenericPoint<float> endPoint = GenericPoint<float>(itr->x, itr->y);
			float endPointToRobot = rayAngle<GenericPoint<float>>(endPoint, GenericPoint<float>(0.0,0.0));
			WorldVector endPointToRobotVector = unitVector(endPointToRobot);
			
			endPoint= endPoint + increment * endPointToRobotVector;
			GenericPoint<float> mapPoint = robotToMap(pose.position, cosRot, sinRot, endPoint);
			while(rayAngle<GenericPoint<float>>(endPoint, GenericPoint<float>(0.0,0.0)) == endPointToRobot)
			{
				ProcessedPointData pointData = processPoint(mapPoint, map);

				float freeProbability = pointData.interpolatedProbability;
				float rotationDerivative = ((-sinRot * endPoint.x - cosRot * endPoint.y) * pointData.gradient.x + 
						   (cosRot * endPoint.x - sinRot * endPoint.y) * pointData.gradient.y);

				dTr(0) -= freeProbability * pointData.gradient.x;
				dTr(1) -= freeProbability * pointData.gradient.y;
				dTr(2) -= freeProbability * rotationDerivative;

				// Calculate the Hessian Diagonals
				H(0, 0) += pow(pointData.gradient.x, 2);
				H(1, 1) += pow(pointData.gradient.y, 2);
				H(2, 2) += pow(rotationDerivative, 2);

				// Calculate the off-diagonals
				H(0, 1) += pointData.gradient.x * pointData.gradient.y;
				H(2, 0) += rotationDerivative * pointData.gradient.x;
				H(2, 1) += rotationDerivative * pointData.gradient.y;

				endPoint= endPoint + increment * endPointToRobotVector;
				mapPoint = robotToMap(pose.position, cosRot, sinRot, endPoint);
			}
		}

		// Copy the off-diagonals (Hessian is symmetric)
		H(0, 1) = H(1, 0);
		H(0, 2) = H(2, 0);
		H(1, 2) = H(2, 1);

		// Note: H is the covariance Matrix!!
		// This has the potential for future use in Kalmann filter or similar
		//covMatrix = invH;

		Vector3f toReturn;
		Matrix3 invH;
		if (inverse(H, invH)) 
		{
			toReturn =  invH * dTr;
		}
		else
		{
			// Hessian is non-inertible for whatever reason
			toReturn = Vector3f::zeros();
		}

		MapPose realReturn(GenericPoint<float>(toReturn(0), toReturn(1)), toReturn(2));
		return realReturn;
	}


	GenericPoint<float> ICPScanMatcher::robotToMap(const GenericPoint<float> position, float cosRot, float sinRot, GenericPoint<float> point) const
	{
		// This implements a rotation matrix
		GenericPoint<float> toReturn = position;
		toReturn.x += cosRot * point.x - sinRot * point.y;
		toReturn.y += sinRot * point.x + cosRot * point.y;
		return toReturn;
	}

	ICPScanMatcher::ProcessedPointData ICPScanMatcher::processPoint(const GenericPoint<float>& mapCoords, const IProbabilityMap& map) const
	{
		IntPoint bottomLeftCorner((int)floor(mapCoords.x), (int)floor(mapCoords.y));
		GenericPoint<float> bottomLeftCornerContinous(bottomLeftCorner);

		// Vectors pointing from corners to the point
		GenericVector<float> bottomLeftOffset = mapCoords - bottomLeftCornerContinous;
		GenericVector<float> upperRightOffset = GenericVector<float>(1.0, 1.0) - bottomLeftOffset;	

		// Get grid values for the 4 grid points surrounding the current mapCoords
		float bottomLeft = map.getProbability(bottomLeftCorner);
		float bottomRight = map.getProbability(IntPoint(bottomLeftCorner.x + 1, bottomLeftCorner.y));
		float upperLeft = map.getProbability(IntPoint(bottomLeftCorner.x, bottomLeftCorner.y + 1));
		float upperRight = map.getProbability(IntPoint(bottomLeftCorner.x + 1, bottomLeftCorner.y + 1));

		// Gradients along x axis
		float bottomXGradient = bottomLeft - bottomRight;
		float upperXGradient = upperLeft - upperRight;

		// Gradients along y axis
		float leftYGradient = bottomLeft - upperLeft;
		float rightYGradient = bottomRight - upperRight;

		ProcessedPointData data;
		
		// Calculate the probability of the point using a weighted average of surrounding points (bilinear interpolation)
		data.interpolatedProbability = 	((upperRightOffset.y) * (bottomLeft * upperRightOffset.x + bottomRight * bottomLeftOffset.x)) +
						((bottomLeftOffset.y) * (upperLeft  * upperRightOffset.x + upperRight  * bottomLeftOffset.x));

		// Drives directly from interpolatedProbability calculation TODO: need to check
		//data.gradient.x = -((bottomXGradient * upperRightOffset.x) + (upperXGradient * bottomLeftOffset.x));
		//data.gradient.y = -((leftYGradient   * upperRightOffset.y) + (rightYGradient * bottomLeftOffset.y));	
		data.gradient.x = -((bottomXGradient * upperRightOffset.y) + (upperXGradient * bottomLeftOffset.y));
		data.gradient.y = -((leftYGradient   * upperRightOffset.x) + (rightYGradient * bottomLeftOffset.x));	

		return data;
	}

}

#include <bob_slam/visualization.h>

#include <bob_toolbox/matrix2.h>
#include <bob_toolbox/dimension2d.h>
#include <bob_toolbox/matrix_operations.h>

namespace bob
{

	MarkerEllipse visualization(const Pose2D& pose, const PoseCovariance& poseCovariance)
	{
		Matrix2 topLeft;

		for (int i = 0; i <= 1; ++i)
		{		
			for (int j = 0; j <= 1; ++j)
			{
				topLeft(i, j) = poseCovariance(i, j);
			}
		}

		EigenValueSolution eigenValues = eigenPairs(topLeft);

		Pose2D vizPose = pose;
		vizPose.theta = eigenValues.first.vector.getAngle();
		
		float width = sqrt(eigenValues.first.value);
		float height = sqrt(eigenValues.second.value);
		
		return MarkerEllipse(vizPose, Dimension2D<float>(width, height));	
	}

}


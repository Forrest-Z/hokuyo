#ifndef _BOB_SLAM_VISUALIZATION_H_
#define _BOB_SLAM_VISUALIZATION_H_

#include <bob_visualization/marker_types.h>

#include <bob_toolbox/pose2d.h>
#include <bob_slam/pose_covariance.h>

namespace bob
{

	MarkerEllipse visualization(const Pose2D& pose, const PoseCovariance& poseCovariance);

}

#endif

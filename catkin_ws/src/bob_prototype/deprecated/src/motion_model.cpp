#include <bob_slam/motion_model.h>
#include <bob_toolbox/world_point.h>
#include <iostream>

#include <bob_toolbox/stat.h>

namespace bob 
{

	Pose2D MotionModel::drawFromMotion(const Pose2D& p, const Pose2D& pnew, const Pose2D& pold) const
	{
		float sxy = 0.3 * srr;
		Pose2D delta = absoluteDifference(pnew, pold);

		Pose2D noisypoint(delta);

		float xSigma = srr * fabs(delta.x) + str * fabs(delta.theta) + sxy * fabs(delta.y);
		float ySigma = srr * fabs(delta.y) + str * fabs(delta.theta) + sxy * fabs(delta.x);
		float thetaSigma = stt * fabs(delta.theta) + srt * sqrt(delta.x * delta.x + delta.y * delta.y);

		noisypoint.x += sampleZeroMeanGaussian(xSigma);
		noisypoint.y += sampleZeroMeanGaussian(ySigma);
		noisypoint.theta += sampleZeroMeanGaussian(thetaSigma);
		noisypoint.theta = fmod(noisypoint.theta, 2 * M_PI);
		if (noisypoint.theta > M_PI)
			noisypoint.theta -= 2 * M_PI;

		return absoluteSum(p, noisypoint);
	}

	Pose2D MotionModel::absoluteDifference(const Pose2D& p1, const Pose2D& p2) const
	{
		Pose2D delta(p1.x - p2.x, p1.y - p2.y, p1.theta - p2.theta);
		delta.theta = atan2(sin(delta.theta), cos(delta.theta));
		float s = sin(p2.theta), c = cos(p2.theta);
		return Pose2D(c * delta.x + s * delta.y, 
				-s * delta.x + c * delta.y, delta.theta);
	}

	Pose2D MotionModel::absoluteSum(const Pose2D& p1,const Pose2D& p2) const
	{
		float s = sin(p1.theta), c = cos(p1.theta);
		return Pose2D(p1.x + c * p2.x - s * p2.y,
				p1.y + s * p2.x + c * p2.y, p1.theta + p2.theta);
	}
}

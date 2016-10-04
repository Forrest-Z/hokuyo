#ifndef _BOB_SLAM_MOTION_MODEL_H_
#define _BOB_SLAM_MOTION_MODEL_H_

#include <bob_toolbox/pose2d.h>

namespace bob 
{ 

	struct MotionModel
	{
		Pose2D drawFromMotion(const Pose2D& p, const Pose2D& pnew, const Pose2D& pold) const;

		MotionModel(float srr, float str, float srt, float stt) : srr(srr), str(str), srt(srt), stt(stt) {}

		private:

		float srr, str, srt, stt;

		Pose2D absoluteSum(const Pose2D& p1,const Pose2D& p2) const;

		Pose2D absoluteDifference(const Pose2D& p1, const Pose2D& p2) const;

	};

}

#endif

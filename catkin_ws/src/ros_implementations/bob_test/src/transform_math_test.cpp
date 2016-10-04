#include <ros/ros.h>
#include <bob_tf/LinearMath/Transform.h>
#include <bob_toolbox/logging.h>
#include <bob_toolbox/pose2d.h>

using namespace bob;

void printTransform(const Transform& transform)
{
	Matrix3x3 R = transform.getBasis();
	Vector3 T = transform.getOrigin();	
	
	LOG_TEST("Rotation:");
	for(int i = 0; i < 3; ++i)
	{
		LOG_TEST(R[i][0] << "," << R[i][1] << "," << R[i][2]);
	}
	
	LOG_TEST("Translation:");
	LOG_TEST(T.getX() << "," << T.getY() << "," << T.getZ());
}

void oldWayTransformCalculation(const Pose2D& odomPose, const Pose2D& estPose)
{
	LOG_TEST("Old way:");
	Transform robotToMap = Transform(createQuaternionFromRPY(0, 0, estPose.theta), Vector3(estPose.x, estPose.y, 0.0)).inverse();
	Transform odomToRobot = Transform(createQuaternionFromRPY(0, 0, odomPose.theta), Vector3(odomPose.x, odomPose.y, 0.0));

	Transform mapToOdom = (odomToRobot * robotToMap).inverse();
	
	printTransform(mapToOdom);
}

void newWayTransformCalculation(const Pose2D& odomPose, const Pose2D& estPose)
{
	LOG_TEST("New way:");
	Pose2D offSet =  odomPose - estPose;
	Transform translation(createQuaternionFromRPY(0, 0, 0), Vector3(offSet.x, offSet.y, 0.0));
	Transform rotation(createQuaternionFromRPY(0, 0, offSet.theta), Vector3(0, 0, 0));
	Transform mapToOdom = translation * rotation;
	printTransform(mapToOdom);
}


int main(int argc, char** argv)
{
	Pose2D odomPose(WorldPoint(1.0, 1.0), 0);
	Pose2D estPose(WorldPoint(0, 1.0), M_PI/4);	

	oldWayTransformCalculation(odomPose, estPose);
	newWayTransformCalculation(odomPose, estPose);	
}

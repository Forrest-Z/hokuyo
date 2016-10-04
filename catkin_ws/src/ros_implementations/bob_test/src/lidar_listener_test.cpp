#include <ros/ros.h>

#include <bob_sensor/itransform_handle.h>

#include <bob_toolbox/logging.h>
#include <bob_ros_implementations/ros_lidar_handle.h>

#include <bob_visualization/visualization.h>
#include <bob_sensor/lidar_scan.h>
#include <bob_lidar/util_functions.h>
#include <bob_toolbox/angular_range.h>
#include <bob_control/unsafe_range_calculator.h>
#include <bob_toolbox/easy_print.h>

#include <iostream>

using namespace bob;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lidar_listener_test");

	/*
	AngularRange range;
	range.add(SimpleAngularRange(0.5, 1.0));
	range.add(SimpleAngularRange(0.8, 0.6));
	LOG_TEST("Data: " << range);

	LOG_TEST(range.isInside(0.8));

	range.add(SimpleAngularRange(0.8, 0.6));
	LOG_TEST("Data: " << range);

	//LOG_TEST("Data: " << range.invert());
	*/

	/*
	ITransformHandle transformHandle;
	IScanSensorHandle lidarHandle(transformHandle);
	ros::Duration(2.0).sleep();

	
	while(ros::ok())
	{
		ros::spinOnce();

		LidarScan scan = lidarHandle.getLidarData();

		PointCloud cloud = toPointCloud(scan);

		UnsafeRangeCalculator unsafeRangeCalculator;

		ros::Time start = ros::Time::now();
		AngularRange range = unsafeRangeCalculator.determineUnsafeRange(scan);

		ros::Duration(0.1).sleep();
	}
	*/

	ros::spin();

	return 0;
}

#ifndef _BOB_ROS_IMPLEMENTATIONS_LIDAR_UTIL_H_
#define _BOB_ROS_IMPLEMENTATIONS_LIDAR_UTIL_H_

#include <bob_sensor/itransform_handle.h>
#include <bob_ros_implementations/ros_lidar_listener.h>
#include <bob_lidar/lidar_configuration.h>

namespace bob
{

	LidarConfiguration determineLidarConfiguration(const ITransformHandle& transformHandle, const ROSLidarListener& lidarListener);

	LidarConfiguration determineLidarConfiguration(const ROSLidarListener& lidarListener, float zeroAngle, bool inverted);

	LidarConfiguration determineLidarConfiguration(const sensor_msgs::LaserScan& msg, float zeroAngle, bool inverted);

}

#endif

#include <ros/ros.h>

#include <bob_visualization/visualization.h>
#include <bob_ros_implementations/ros_transform_handle.h>

#include <bob_visualization/bob_toolbox_visualization.h>
#include <bob_visualization/marker_types.h>
#include <bob_sensor/itransform_handle.h>

#include <bob_ros_implementations/ros_lidar_handle.h>
#include <bob_lidar/util_functions.h>
#include <bob_lidar/passable_range.h>

#include <bob_config/config.h>

#include <bob_wall_following/wall_follow_heading_chooser.h>
#include <bob_control/wall_follow_side.h>

using namespace bob;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lidar_listener_test");
	ros::NodeHandle nh;

	/*	

	ROSTransformHandle transformHandle;
	ROSLidarHandle lidarHandle(transformHandle);
	ros::Duration(2.0).sleep();

	
	while(ros::ok())
	{
		ros::spinOnce();
	
		WallFollowHeadingChooser wallFollowHeadingChooser(lidarHandle, RightSide);
		float angle = wallFollowHeadingChooser.getHeadingInRobotFrame();

		std::vector<WorldPoint> line;
		line.push_back(WorldPoint(0,0));
		line.push_back(WorldPoint(cos(angle), sin(angle)));
		visualizer->visualize("angle", visualization(MarkerLine(line)));

		ros::Duration(0.1).sleep();
	}

	*/
	ros::spin();

	return 0;
}

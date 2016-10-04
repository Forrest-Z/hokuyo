#ifndef _BOB_GRID_MAP_TO_ROS_H_
#define _BOB_GRID_MAP_TO_ROS_H_

#include <nav_msgs/OccupancyGrid.h>

#include <bob_ros_implementations/grid_map_visualization.h>
#include <bob_grid_map/map_metadata.h>

#include <bob_toolbox/world_point.h>

namespace bob
{

	//! Converts the map data into a message so it can be sent over ROS topic
	template <typename MapType>
	nav_msgs::OccupancyGrid convertMapToROS(const MapType& map)
	{
		nav_msgs::OccupancyGrid msg;
		float resolution = map.getResolution();

		msg.header.frame_id = "map";
		msg.header.stamp = ros::Time::now();
		msg.info.resolution = resolution;

		msg.info.width = map.getMapMetadata().bounds.width;
		msg.info.height = map.getMapMetadata().bounds.height;

		WorldPoint originPoint = map.mapToWorld(map.getMapMetadata().bottomLeftCorner);

		// We subtract resolution / 2 because the map location is in 
		// the exact center of cell, while ROS expects corner
		msg.info.origin.position.x = originPoint.x - resolution / 2;
		msg.info.origin.position.y = originPoint.y - resolution / 2;

		// Map is 2D and orientation never changes
		msg.info.origin.position.z = 0.0;
		msg.info.origin.orientation.w = 1.0;

		// Copy in the raw data, after conerting
		msg.data = ROSMapData(map);
		return msg;
	}

}

#endif

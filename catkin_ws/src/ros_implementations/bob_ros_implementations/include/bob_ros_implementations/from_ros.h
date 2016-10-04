#ifndef _BOB_GRID_MAP_FROM_ROS_H_
#define _BOB_GRID_MAP_FROM_ROS_H_

#include <nav_msgs/OccupancyGrid.h>

namespace bob
{

	class IProbabilityMap;
	class MapMetadata;

	//! This header defines some useful functions used to transfer data
	//! from ROS into our datatypes.
	typedef nav_msgs::OccupancyGridConstPtr MapMsgPtr;

	//! Extracts the size data from a ROS map message
	MapMetadata extractMsgMetadata(const MapMsgPtr& new_map);

	//! Imports ROS map data into a map object

	//! Works by copying the data element by element after conversion
	//! Templated so that it works with all types of RawMap
	void importROSMapData(IProbabilityMap& costmap, const MapMsgPtr& new_map);

}

#endif

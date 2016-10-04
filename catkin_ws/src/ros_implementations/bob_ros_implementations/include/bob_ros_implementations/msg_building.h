#ifndef _BOB_VISUALIZATION_MSG_BUILDING_H_
#define _BOB_VISUALIZATION_MSG_BUILDING_H_

#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

namespace bob
{

	// Forward declaration
	class MarkerStyle;

	visualization_msgs::Marker defaultFromStyle(const MarkerStyle& style);

	std_msgs::ColorRGBA getColorMsg(const MarkerStyle& style);

}

#endif

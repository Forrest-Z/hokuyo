#include <bob_ros_implementations/msg_building.h>

#include <bob_visualization/marker_style.h>

namespace bob
{

	visualization_msgs::Marker defaultFromStyle(const MarkerStyle& style)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time::now();
		marker.action = visualization_msgs::Marker::ADD;
		marker.ns = "marker_manager";	
		marker.id = 0;
		marker.color = getColorMsg(style);
		marker.scale.x = style.width;
		marker.scale.y = style.width;

		// Never auto-delete
		marker.lifetime = ros::Duration();

		return marker;
	}


	std_msgs::ColorRGBA getColorMsg(const MarkerStyle& style)
	{
		std_msgs::ColorRGBA colorMsg;
		colorMsg.r = style.color->r();
		colorMsg.b = style.color->b();
		colorMsg.g = style.color->g();
		colorMsg.a = style.alpha;
		return colorMsg;
	}



}


#include <bob_ros_implementations/ros_visualizer.h>

#include <bob_toolbox/world_point.h>

#include <bob_visualization/marker_types.h>
#include <bob_visualization/visualization.h>

#include <bob_ros_implementations/msg_building.h>

#include <geometry_msgs/Point.h>

#include <vector>

namespace bob
{

	geometry_msgs::Point ROSPoint(const WorldPoint& point)
	{
		geometry_msgs::Point p;
		p.x = point.x;
		p.y = point.y;
		p.z = 0;
		return p;
	}

	void configureROSVisualization()
	{
		visualizer.reset(new ROSVisualizer());
	}

	void ROSVisualizer::visualize(std::string title, const MarkerArrow& subject, const MarkerStyle& style)
	{
		visualization_msgs::Marker marker = defaultFromStyle(style);
		uint32_t shape = visualization_msgs::Marker::ARROW;

		marker.points.push_back(ROSPoint(subject.from));
		marker.points.push_back(ROSPoint(subject.to));

		// Shaft diameter
		marker.scale.x = 0.01;

		// Head diameter
		marker.scale.y = 0.03;

		publishMarker(title, marker);
	}

	void ROSVisualizer::visualize(std::string title, const MarkerLine& subject, const MarkerStyle& style)
	{
		visualization_msgs::Marker marker = defaultFromStyle(style);
		marker.type = visualization_msgs::Marker::LINE_STRIP;

		for (auto& point : subject.data)
		{
			marker.points.push_back(ROSPoint(point));
		}	

		publishMarker(title, marker);
	}

	void ROSVisualizer::visualize(std::string title, const MarkerLineList& subject, const MarkerStyle& style)
	{
		visualization_msgs::Marker marker = defaultFromStyle(style);
		marker.type = visualization_msgs::Marker::LINE_LIST;

		for (auto& point : subject.data)
		{
			marker.points.push_back(ROSPoint(point));
		}	

		publishMarker(title, marker);
	}

	void ROSVisualizer::visualize(std::string title, const MarkerSquares& subject, const MarkerStyle& style)
	{
		visualization_msgs::Marker marker = defaultFromStyle(style);

		marker.type = visualization_msgs::Marker::CUBE_LIST;
		marker.scale.x = subject.width;
		marker.scale.y = subject.width;

		for (auto& point : subject.data)
		{
			marker.points.push_back(ROSPoint(point));
		}

		publishMarker(title, marker);
	}

	void ROSVisualizer::publishMarker(const std::string& topicName, const visualization_msgs::Marker& marker)
	{
		if (publishers.count(topicName) == 0)
		{
			ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>(topicName, 1, true);
			std::pair<std::string, ros::Publisher> newEntry(topicName, marker_pub); 
			publishers.insert(newEntry);
		}
		publishers[topicName].publish(marker);
	}

}


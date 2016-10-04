#ifndef _BOB_ROS_IMPLEMENTATIONS_ROS_VISUALIZER_H_
#define _BOB_ROS_IMPLEMENTATIONS_ROS_VISUALIZER_H_


#include <bob_visualization/visualizer.h>
#include <bob_visualization/marker_style.h>
#include <bob_visualization/marker_types.h>

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>

#include <unordered_map>

namespace bob
{

	//! \broef ROS implementation of visualization engine
	class ROSVisualizer : public Visualizer
	{

		public:

			virtual void visualize(std::string title, const MarkerLine& subject, const MarkerStyle& style = MarkerStyle());

			virtual void visualize(std::string title, const MarkerLineList& subject, const MarkerStyle& style = MarkerStyle());

			virtual void visualize(std::string title, const MarkerSquares& subject, const MarkerStyle& style = MarkerStyle());

			virtual void visualize(std::string title, const MarkerArrow& subject, const MarkerStyle& style = MarkerStyle());

		private:

			//! \brief Publish a marker onto a topics
			//! \param topicName The name of the topic
			//! \param marker The marker to publish
			void publishMarker(const std::string& topicName, const visualization_msgs::Marker& marker);

			//! Connection to ROS system
			ros::NodeHandle n;

			//! Map from topic name to publishers
			std::unordered_map<std::string, ros::Publisher> publishers;	
	};

	//! \brief Configure the global visualizer object so that it uses a ROS visualization engine
	void configureROSVisualization();

	geometry_msgs::Point ROSPoint(const WorldPoint& point);

}

#endif

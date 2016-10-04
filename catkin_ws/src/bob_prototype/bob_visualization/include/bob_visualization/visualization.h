#ifndef _BOB_VISUALIZATION_ROS_VISUALIZATION_H_
#define _BOB_VISUALIZATION_ROS_VISUALIZATION_H_

#include <bob_visualization/visualizer.h>

// Added here for convienience
#include <bob_visualization/marker_style.h>
#include <bob_visualization/marker_types.h>
#include <bob_visualization/bob_toolbox_visualization.h>

namespace bob
{

	//! Global variable exposing handle to visualization
	extern std::unique_ptr<Visualizer> visualizer;

}

#endif

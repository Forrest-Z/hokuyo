#ifndef _BOB_TOOLBOX_VISUALIZATION_H_
#define _BOB_TOOLBOX_VISUALIZATION_H_

#include <bob_visualization/marker_types.h>

// This file is defined here instead of bob_toolbox to prevent a circular dependency
// between bob_visualization and bob_toolbox.

namespace bob
{

	class SimpleAngularRange;
	class AngularRange;

	//! \brief Convert a SimpleAngularRange to a MarkerLine for visualization purposes
	//! \param range The SimpleAngularRange to convert
	//! \return A MarkerLine that can be visualized using a Visualizer
	MarkerLine visualization(const SimpleAngularRange& range);

	//! \brief Convert an AngularRange to a MarkerLine for visualization purposes
	//! \param range The AngularRange to convert
	//! \return A MarkerLine that can be visualized using a Visualizer
	MarkerLine visualization(const AngularRange& range);

}

#endif

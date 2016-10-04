#ifndef _BOB_COVERAGE_AREA_CHOOSER_H_
#define _BOB_COVERAGE_AREA_CHOOSER_H_

#include <bob_coverage/discrete_area.h>
#include <bob_coverage/coverage_tracker.h>
#include <bob_grid_map/costmap.h>

#include <bob_sensor/itransform_handle.h>

namespace bob
{

	class AreaChooser
	{

		public:

			AreaChooser(Costmap& costmap, const ITransformHandle& transformHandle); 

			//! Suggests the next area for executing coverage
			//! 
			//! area - The area that is returned
			//
			//! Returns:
			//! true if area could be found
			DiscreteArea nextArea() const;

		private:

			//! Tracks the robot's position and marks the area
			CoverageTracker coverageTracker;

			Costmap& costmap;

			const ITransformHandle& transformHandle;

			std::vector<WorldPoint> boundRegion;
	};

}

#endif

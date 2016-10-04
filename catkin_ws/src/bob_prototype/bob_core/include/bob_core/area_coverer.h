#ifndef _BOB_CORE_AREA_COVERER_H_
#define _BOB_CORE_AREA_COVERER_H_

#include <bob_coverage/icoverage_algorithm.h>
#include <bob_coverage/coverage_tracker.h>
#include <bob_coverage/discrete_area.h>
#include <bob_core/area_processor.h>
#include <bob_sensor/isensor_handle.h>

namespace bob
{
	//! \brief A Class provides a function to use user specified coverage algorithm 
	//! cover given area. 
	class AreaCoverer
	{

		public:
			//! \brief Constructor
			//! \param coverageAlgorithm The algorithm will be used to 
			//! cover area (e.g.Boustrophedon, STC).
			//! \param areaProcessor Tracks explored area and covered area, 
			//! and can process these areas to get coverable area
			//! \param sensorHandle Give access to sensor data.			
			AreaCoverer(ICoverageAlgorithm& coverageAlgorithm, AreaProcessor& areaProcessor, const ISensorHandle& sensorHandle) : 
			coverageAlgorithm(coverageAlgorithm),
			areaProcessor(areaProcessor),
			sensorHandle(sensorHandle)
			{}

			//! \brief A function can be called to cover given area.
			//! \param area Area to cover
			void cover(DiscreteArea area);

		private:

			
			void coverSubsection(DiscreteArea area);

			ICoverageAlgorithm& coverageAlgorithm;
			AreaProcessor& areaProcessor;
			const ISensorHandle& sensorHandle;

	};

}

#endif

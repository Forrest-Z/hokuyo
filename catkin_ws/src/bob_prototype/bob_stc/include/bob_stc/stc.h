#ifndef _BOB_STC_STC_H_
#define _BOB_STC_STC_H_

#include <bob_coverage/discrete_area.h>
#include <bob_coverage/icoverage_algorithm.h>

namespace bob
{

	class LockableMap;
	class ControlHandle;
	class ISensorHandle;

	//! \brief Runs a Spanning Tree Coverage algorithm. Implements ICoverageAlgorithm.
	class STC : public ICoverageAlgorithm
	{

		public:
	
			//! \brief Create an STC object
			//! \param lockableMap Provides access to SLAM map
			//! \param controlHandle Provides connection to control robot behavior
			//! \param sensorHandle Provides connection to robot sensor information
			STC(const LockableMap& lockableMap, ControlHandle& controlHandle, const ISensorHandle& sensorHandle) :
			lockableMap(lockableMap),
			controlHandle(controlHandle),
			sensorHandle(sensorHandle)
			{}

			//! See documentation in ICoverageAlgorithm
			virtual void coverArea(const DiscreteArea& area);

		private:

			//! Provides access to SLAM map
			const LockableMap& lockableMap;	

			//! Provides connection to control robot behavior
			ControlHandle& controlHandle;

			//! Provides connection to robot sensor information
			const ISensorHandle& sensorHandle;

	};

}

#endif

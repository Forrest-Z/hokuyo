#include <bob_coverage/coverable_area_tracker.h>
#include <bob_toolbox/easy_print.h>

namespace bob
{

	void CoverableAreaTrackerThread::operator()()
	{
		MapLocationSet newArea = frontierTracker.getExploredArea();		
		DiscreteArea coveredArea = coverageTracker.getCoveredArea();
		for (DiscreteArea::iterator itr = coveredArea.begin(); itr != coveredArea.end(); ++itr)
		{
			newArea.erase(*itr);
		}
		coverableArea.setValue(DiscreteArea(0.05, newArea.begin(), newArea.end()));		
	}

	DiscreteArea CoverableAreaTracker::getCoverableSpace() const
	{
		return trackerThread.getCoverableSpace();
	}

	DiscreteArea CoverableAreaTrackerThread::getCoverableSpace() const
	{
		return coverableArea.getValue();
	}

}


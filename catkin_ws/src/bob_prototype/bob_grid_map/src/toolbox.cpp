#include <bob_grid_map/toolbox.h>

#include <bob_grid_map/lockable_map.h>

#include <bob_toolbox/strict_lock.h>
#include <bob_system/system_utilities.h>

namespace bob
{

	void waitForMapAvailable(const LockableMap& lockableMap, float pollingPeriod)
	{
		while (systemUtilities->ok())
		{
			{
				StrictLock lock(lockableMap.getLock());
				const Costmap& map = lockableMap.getLockedResource(lock);
				if (map.isMapAvailable())
					return;
			}

			// Sleep after releasing map lock
			systemUtilities->sleep(pollingPeriod);
		}
	}

}


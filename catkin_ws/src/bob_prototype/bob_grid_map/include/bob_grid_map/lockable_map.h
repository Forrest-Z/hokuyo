#ifndef _BOB_GRID_MAP_LOCKABLE_MAP_H_
#define _BOB_GRID_MAP_LOCKABLE_MAP_H_

#include <bob_config/config.h>
#include <bob_grid_map/costmap.h>
#include <bob_toolbox/externally_locked.h>

namespace bob
{

	//! Provides a costmap with external mutex locking

	//! Usage:
	//! LockableMap lockMap;
	//! {
	//! 	StrickLock lock(lockMap.getLock());
	//! 	Costmap& map = getLockedResource(lock);
	//! someAlgorithm(map);
	//! etc...
	//
	//! } Lock released
	class LockableMap : public ExternallyLocked<Costmap>
	{

		public:

			LockableMap() : 
			ExternallyLocked(new Costmap(Config::MAP_RESOLUTION))
			{}

			const LocationMapper& getLocationMapper() const
			{
				return *lockedData;
			}

	};

}

#endif

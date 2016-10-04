#ifndef _BOB_GRID_MAP_TOOLBOX_H_
#define _BOB_GRID_MAP_TOOLBOX_H_


namespace bob
{
	class LockableMap;

	//! \brief Blocks and polls the map until it is available, then unblocks.
	//! \param map The map to wait for
	//! \param pollingPeriod The time between polling
	void waitForMapAvailable(const LockableMap& map, float pollingPeriod = 0.1);

}

#endif

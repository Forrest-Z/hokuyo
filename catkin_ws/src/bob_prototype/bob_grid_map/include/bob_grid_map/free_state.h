#ifndef _BOB_GRID_MAP_FREE_STATE_H_
#define _BOB_GRID_MAP_FREE_STATE_H_

namespace bob
{

	//! Used to determine the occupancy status of a point in the map
	enum FreeState
	{
		//! Free from obstacles
		Free,
		
		//! An obstacle
		SensedObstacle,

		HiddenObstacle,

		//! Unknown, not visited or sensed
		Unknown
	};

}

#endif

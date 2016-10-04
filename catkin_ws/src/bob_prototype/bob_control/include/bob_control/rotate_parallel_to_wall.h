#ifndef _BOB_CONTROL_ROTATE_PARALLEL_TO_WALL_H_
#define _BOB_CONTROL_ROTATE_PARALLEL_TO_WALL_H_

#include <bob_control/simple_commander.h>

#include <bob_sensor/itransform_handle.h>

namespace bob
{

	class RotateParallelToWall
	{
		public:

			RotateParallelToWall(SimpleCommander& commander, IScanSensorHandle& lidarHandle, const ITransformHandle& transformHandle, WallFollowSide side) :
				commander(commander),
				lidarHandle(lidarHandle),
				transformHandle(transformHandle),
				side(side)
			{}
			
			void run();
		
		private:
			
			SimpleCommander& commander;

			IScanSensorHandle& lidarHandle;
			
			const ITransformHandle& transformHandle;

			WallFollowSide side;
		
		

	};

}

#endif

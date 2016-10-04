#ifndef _BOB_BOUSTROPHEDON_BOUSTROPHEDON_EXECUTOR_H_
#define _BOB_BOUSTROPHEDON_BOUSTROPHEDON_EXECUTOR_H_

#include <bob_coverage/discrete_area.h>
#include <bob_boustrophedon/boustrophedon.h>
#include <bob_coverage/area_chooser.h>

#include <bob_control_handle/control_handle.h>
#include <bob_sensor/isensor_handle.h>

namespace bob
{

	// NOTE: Not currently in use

	//! Uses an AreaChooser to cover all of the visible areas in the map. 
	class BoustrophedonExecutor
	{

		public:

			//! Constructor
			BoustrophedonExecutor(ControlHandle& controlHandle, const ISensorHandle& sensorHandle, AreaChooser& areaChooser): 
				boustrophedon(controlHandle, sensorHandle),
				areaChooser(areaChooser)
		{}

			//! Start Boustrophedon.
			void execute();

		private:

			Boustrophedon boustrophedon;
			AreaChooser& areaChooser;

			

	};

}

#endif

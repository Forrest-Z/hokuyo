#include <bob_frontier_exploration/frontier_explorer.h>
#include <bob_toolbox/geometry.h>


namespace bob
{

	void FrontierExplorer::exploreFrontiers()
	{	
		frontierTracker.waitForInitialization();

		FrontierChooser::MapWorldPair exploreGoal;

		
		while(frontierChooser.next(exploreGoal))
		{
			FrontierCondition frontierCondition(frontierTracker, exploreGoal.location);		
			navigationManager.navigationGoal(exploreGoal.point, frontierCondition);	
		}

		// Slow down		
		Pose2D currPose = transformHandle.getLocalizedPose();
		AccelerateCommand slowdown(Line(currPose, currPose.theta), 0.05, 0.0);
		simpleCommander.execute(slowdown);
	}

}


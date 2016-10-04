#include <bob_stc/stc_path_combiner.h>

#include <bob_toolbox/line_geometry.h>
#include <bob_stc/easy_print.h>
#include <bob_toolbox/easy_print.h>

#include <limits>

namespace bob
{

	STCPlan STCPathCombiner::combinePaths(std::list<STCPath> paths) const
	{
		STCPlan result;

		if (paths.size() == 1)
		{
			// Don't want the code to crash if there is only one path
			// So return the path in a sort of "degenerate" plan
			result.push_back(paths.front());
		}
		else if (paths.size() > 1)
		{
			// First two elements are a special case, because they're both loops
			addFirstTwoElements(result, paths);

			while (!paths.empty())
			{
				findAndAddElement(result, paths);
			}
		}

		return result;
	}

	void STCPathCombiner::findAndAddElement(STCPlan& plan, std::list<STCPath>& newSTCPaths) const
	{
		float shortestDistance = std::numeric_limits<float>::max();
		std::list<STCPath>::iterator newPathToAdd;
		std::list<STCPath>::iterator existingPathToConnect;
		STCConnection connectionDetails;
		
		for (std::list<STCPath>::iterator existingPathsItr = plan.begin();
				existingPathsItr != plan.end(); ++existingPathsItr)
		{
			for (std::list<STCPath>::iterator newPathsItr = newSTCPaths.begin();
					newPathsItr != newSTCPaths.end(); ++newPathsItr)
			{
				STCConnection connection = shortestConnection(*newPathsItr, *existingPathsItr);
				float segmentLength = diagonalDistance(connection.firstSegmentPoint, connection.secondSegmentPoint);
				if (segmentLength < shortestDistance)
				{
					shortestDistance = segmentLength;
					newPathToAdd = newPathsItr;
					existingPathToConnect = existingPathsItr;
					connectionDetails = connection;
				}
			}
		}

		std::pair<STCPath, STCPath> splitPath = splitAroundNewPoint(*existingPathToConnect, connectionDetails.secondPointBefore, connectionDetails.secondSegmentPoint);		
	
		rearrangeAroundNewPoint(*newPathToAdd, connectionDetails.firstPointBefore, connectionDetails.firstSegmentPoint);

		std::list<STCPath>::iterator insertionPoint = plan.erase(existingPathToConnect);

		if (splitPath.second.size() > 1)
			insertionPoint = plan.insert(insertionPoint, splitPath.second);

		insertionPoint = plan.insert(insertionPoint, *newPathToAdd);

		if (splitPath.first.size() > 1)
			insertionPoint = plan.insert(insertionPoint, splitPath.first);

		newSTCPaths.erase(newPathToAdd);
	}

	void STCPathCombiner::addFirstTwoElements(STCPlan& plan, std::list<STCPath>& newSTCPaths) const
	{
		float shortestDistance = std::numeric_limits<float>::max();
		std::list<STCPath>::iterator newPathToAdd;
		STCConnection connectionDetails;

		// Note we are starting after begin because we are comparing with the front()
		for (std::list<STCPath>::iterator newPathsItr = ++newSTCPaths.begin();
				newPathsItr != newSTCPaths.end(); ++newPathsItr)
		{
			STCConnection connection = shortestConnection(*newPathsItr, newSTCPaths.front());
			float segmentLength = diagonalDistance(connection.firstSegmentPoint, connection.secondSegmentPoint);
			if (segmentLength < shortestDistance)
			{
				shortestDistance = segmentLength;
				newPathToAdd = newPathsItr;
				connectionDetails = connection;
			}
		}

		rearrangeAroundNewPoint(newSTCPaths.front(), connectionDetails.secondPointBefore, connectionDetails.secondSegmentPoint);
		rearrangeAroundNewPoint(*newPathToAdd, connectionDetails.firstPointBefore, connectionDetails.firstSegmentPoint);

		plan.push_back(newSTCPaths.front());
		plan.push_back(*newPathToAdd);

		newSTCPaths.erase(newPathToAdd);
		newSTCPaths.erase(newSTCPaths.begin());
	}

	STCPathCombiner::STCConnection STCPathCombiner::shortestConnection(STCPath& first, STCPath& second) const
	{
		STCConnection result;

		float shortestLength = std::numeric_limits<float>::max();

		// Iterate over both paths, but not the last point
		// This is because we are considering the segments from the current point to the next point
		// so we can't use the last point because it has no next point.
		for (STCPath::iterator firstItr = first.begin(); firstItr != --(first.end()); ++firstItr)	
		{
			for (STCPath::iterator secondItr = second.begin(); secondItr != (--second.end()); ++secondItr)
			{
				// Getting iterators to the next element
				STCPath::iterator nextFirstItr = firstItr;
				STCPath::iterator nextSecondItr = secondItr;
				++nextFirstItr;
				++nextSecondItr;

				// Determining the segment ahead of the point
				LineSegment firstSegment(*firstItr, *nextFirstItr);
				LineSegment secondSegment(*secondItr, *nextSecondItr);

				// Get the shortest connection between the segments
				SegmentLengthPair pair = shortestLineBetweenSegments(firstSegment, secondSegment);
				if (pair.length < shortestLength)
				{
					shortestLength = pair.length;

					// We can be sure the first point will lie on first path
					// and the second point lies on second path
					// (based on design of shortestLineBetweenSegments())
					result.firstSegmentPoint = pair.segment.first;
					result.secondSegmentPoint = pair.segment.second;

					result.firstPointBefore = firstItr;
					result.secondPointBefore = secondItr;
				}	
			}
		}

		return result; 
	}

	
	// Insert the new point:
	//
	// path.begin() -> *-----*
	//                 |     |
	//                 *-----*
	//                  
	// After insertion:
	//                 *-----*
	//                 |     |       
	//                 *--*--*
	//                    ^ new point
	//                    |
	//                    path.begin()
	void STCPathCombiner::rearrangeAroundNewPoint(STCPath& path, STCPath::iterator pointBefore, WorldPoint newPoint) const
	{
		// Assume pointBefore cannot point to end()
		STCPath::iterator pointAfter = pointBefore;
		++pointAfter;

		// Move the first section to the end, break the path at desired position to 
		// make a room for the new point.
		// If path is a loop
		if (path.front() == path.back())
		{
			// Move first section (start from ++path.begin() to the point before the new point)
			// to the end.
			path.insert(path.end(), ++path.begin(), pointAfter);
		}	
		else
		{
			// Move first section (start from path.begin() to the point before the new point)
			// to the end.
			path.insert(path.end(), path.begin(), pointAfter);
		}
		path.erase(path.begin(), pointAfter);

		// Insert the new point if it doesn't exist already (was segment point)
		if (path.front() != newPoint)
			path.insert(path.begin(), newPoint);

		if (path.back() != newPoint)
			path.insert(path.end(), newPoint);
	}

	std::pair<STCPath, STCPath> STCPathCombiner::splitAroundNewPoint(const STCPath& path, STCPath::const_iterator pointBefore, WorldPoint newPoint) const
	{
		// Assume pointBefore cannot point to end()
		STCPath::const_iterator nextPoint = pointBefore;
		++nextPoint;

		std::pair<STCPath, STCPath> result;
		result.first.insert(result.first.end(), path.begin(), nextPoint);
		result.second.insert(result.second.end(), nextPoint, path.end());

		// Add the newPoint, but only if it doesn't result in duplicate point
		if (result.first.back() != newPoint)
			result.first.insert(result.first.end(), newPoint);

		if (result.second.front() != newPoint)
			result.second.insert(result.second.begin(), newPoint);

		return result;
	}
}


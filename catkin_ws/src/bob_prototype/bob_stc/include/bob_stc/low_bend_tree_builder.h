#ifndef _BOB_STC_LOW_BEND_TREE_BUILDER_H_
#define _BOB_STC_LOW_BEND_TREE_BUILDER_H_

#include <bob_stc/tree_builder.h>
#include <bob_stc/grid_set.h>
#include <boost/bimap.hpp>
#include <bob_stc/grid_point.h>
#include <unordered_set>
#include <boost/bimap/unordered_set_of.hpp>
#include <boost/bimap/unordered_multiset_of.hpp>
#include <bob_toolbox/direction.h>
#include <bob_stc/grid_tile_type.h>
#include <bob_stc/spanning_tree_grid.h>

using namespace boost::bimaps;

namespace bob
{

	class SpanningTreeGrid;
	class GridPoint;
	enum Direction;

	class LowBendTreeBuilder : public TreeBuilder
	{

		public:

			virtual SpanningTreeGrid buildTree(GridSet freeCells);

		private:

			typedef boost::bimap<unordered_set_of<GridPoint, GridPointHash>, unordered_multiset_of<const GridTileType*> > tileTypeBimap;
		
			tileTypeBimap tileTypes;

			GridSet remainingTiles;

			std::set<Direction> directionOfWalls(GridPoint point);

			std::set<Direction> forcingDirections(GridPoint point);

			const GridTileType* tileFromWalls(std::set<Direction> walls);

			//void printTree();

			void initializeTilesWithWalls();

			void chooseOrientation();

			void propagateLeafs();

			const GridTileType* typeAfterConnections(const GridTileType* oldType, std::set<Direction> connections);

			void growTreeWithoutChanging(SpanningTreeGrid& tree);

			bool growTreeOnce(SpanningTreeGrid& tree, bool allowLocalChange, bool allowOutsideChange);

			GridPoint chooseParentPoint();

			SpanningTreeGrid buildTreeFromTypes();

			//! These are objects used to represent tile types. Their references are used kind of like enums.
			//! Look up "state design pattern" online for some more explanation
			Wild wild;
			Cross cross;
			Horizontal horizontal;
			Vertical vertical;
			Horizontal horizontalBridge;
			Vertical verticalBridge;
			Two two;
			Leaf leaf;

	};

}

#endif

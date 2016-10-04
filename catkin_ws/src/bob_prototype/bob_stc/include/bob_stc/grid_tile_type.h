#ifndef _BOB_STC_GRID_TILE_TYPE_H_
#define _BOB_STC_GRID_TILE_TYPE_H_

#include <bob_toolbox/direction.h>
#include <set>
#include <string>

namespace bob
{

//! This is an abstract base class used for state pattern
//! derived classes are states
class GridTileType
{

	//! Prevent accidental copying
	GridTileType(GridTileType const &);
	void operator=(GridTileType const &);

public:

	GridTileType() {};

	//! A set of possible connections which can be satisfied by the 
	//! tile without having to change it's type.
	//! Default is to support all directions
	virtual const std::set<Direction> possibleConnections() const;

	//! For debugging purposes
	virtual const std::string representation() const = 0;

};

class Wild : public GridTileType
{

	virtual const std::string representation() const { return "?"; };

};

class Cross : public GridTileType
{

	virtual const std::string representation() const { return "+"; };

};

class Horizontal : public GridTileType
{

	//! Can only support horizontal connections
	virtual const std::set<Direction> possibleConnections() const;

	virtual const std::string representation() const { return "-"; };

};

class Vertical : public GridTileType
{

	//! Can only support vertical connections
	virtual const std::set<Direction> possibleConnections() const;

	virtual const std::string representation() const { return "|"; };

};

class Two : public GridTileType
{

	virtual const std::string representation() const { return "2"; };

};

class Leaf : public GridTileType
{

	virtual const std::string representation() const { return "'"; };

};

}

#endif

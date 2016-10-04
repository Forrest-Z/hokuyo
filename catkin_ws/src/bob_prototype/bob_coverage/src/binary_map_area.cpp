#include <bob_coverage/binary_map_area.h>

namespace bob
{

	bool BinaryMapArea::contains(MapLocation mapLocation) const
	{
		return (*this)[mapLocation];
	}

	void BinaryMapArea::insert(const MapLocation& toInsert)
	{
		(*this)[toInsert] = true;
	}

	bool BinaryMapArea::defaultCellValue() const
	{
		return false;
	}
}


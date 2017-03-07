#include <srslib_framework/localization/map/logical/LogicalMap.hpp>

#include <limits>

#include <srslib_framework/datastructure/Location.hpp>
#include <srslib_framework/utils/Filesystem.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap::LogicalMap(LogicalMetadata metadata, WeightedGrid2d* grid) :
    BaseMap(grid, metadata.widthM, metadata.heightM, metadata.resolution, metadata.origin),
    metadata_(metadata)
{}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMap::addLabeledArea(Rectangle surface, string label, shared_ptr<MapNotes> notes)
{
    LabeledArea labeledArea;
    labeledArea.surface = surface;
    labeledArea.label = label;
    labeledArea.notes = notes;

    labeledAreas_[label] = labeledArea;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMap::checkAreas(unsigned int cCells, unsigned int rCells,
    LabeledAreaMapType& areas) const
{
    areas.clear();

    for (auto area : labeledAreas_)
    {
        if (area.second.surface.isIn(cCells, rCells))
        {
            areas[area.first] = area.second;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMap::checkAreas(double xM, double yM, LabeledAreaMapType& areas) const
{
    unsigned int c;
    unsigned int r;
    transformM2Cells(xM, yM, c, r);

    checkAreas(c, r, areas);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMap::costSet(unsigned int cCells, unsigned int rCells, WeightedGrid2d::BaseType cost)
{
    getGrid()->payloadSet(Location(cCells, rCells), cost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMap::costMax(unsigned int cCells, unsigned int rCells, WeightedGrid2d::BaseType cost)
{
    getGrid()->payloadMax(Location(cCells, rCells), cost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool LogicalMap::getNeighbor(const Position& position, Position& result)
{
    return getGrid()->getNeighbor(position, result);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
WeightedGrid2d::BaseType LogicalMap::getCost(const Position& position) const
{
    return getGrid()->getPayload(position);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
WeightedGrid2d::BaseType LogicalMap::getCost(unsigned int cCells, unsigned int rCells) const
{
    return getGrid()->getPayload(Location(cCells, rCells));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
WeightedGrid2d::BaseType LogicalMap::getWeight(const Position& position) const
{
    return getGrid()->getWeight(position);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMap::getWeights(unsigned int cCells, unsigned int rCells,
    WeightedGrid2d::BaseType& north, WeightedGrid2d::BaseType& northEast,
    WeightedGrid2d::BaseType& east, WeightedGrid2d::BaseType& southEast,
    WeightedGrid2d::BaseType& south, WeightedGrid2d::BaseType& southWest,
    WeightedGrid2d::BaseType& west, WeightedGrid2d::BaseType& northWest)
{
    return getGrid()->getWeights(Location(cCells, rCells),
        north, northEast,
        east, southEast,
        south, southWest,
        west, northWest);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
WeightedGrid2d::BaseType LogicalMap::getWeights(unsigned int cCells, unsigned int rCells,
    int orientation) const
{
    return getGrid()->getWeight(Position(cCells, rCells, orientation));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
ostream& operator<<(ostream& stream, const LogicalMap& map)
{
    stream << "LogicalMap" << endl;
    stream << *map.getGrid() << endl;

    for (auto area : map.labeledAreas_)
    {
        stream << area.second << endl;
    }

    return stream;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool operator==(const LogicalMap& lhs, const LogicalMap& rhs)
{
    if (&lhs == &rhs)
    {
        return true;
    }

    if (lhs.metadata_ == rhs.metadata_ && *lhs.getGrid() == *rhs.getGrid())
    {
        if (lhs.labeledAreas_.size() != rhs.labeledAreas_.size())
        {
            return false;
        }

        for (auto area : lhs.labeledAreas_)
        {
            if (rhs.labeledAreas_.find(area.first) == rhs.labeledAreas_.end())
            {
                return false;
            }
        }

        for (auto area : rhs.labeledAreas_)
        {
            if (lhs.labeledAreas_.find(area.first) == lhs.labeledAreas_.end())
            {
                return false;
            }
        }

        return true;
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMap::setWeights(unsigned int cCells, unsigned int rCells,
    WeightedGrid2d::BaseType north, WeightedGrid2d::BaseType northEast,
    WeightedGrid2d::BaseType east, WeightedGrid2d::BaseType southEast,
    WeightedGrid2d::BaseType south, WeightedGrid2d::BaseType southWest,
    WeightedGrid2d::BaseType west, WeightedGrid2d::BaseType northWest)
{
    getGrid()->setWeights(Location(cCells, rCells),
        north, northEast,
        east, southEast,
        south, southWest,
        west, northWest);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods
//

} // namespace srs

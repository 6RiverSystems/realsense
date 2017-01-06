#include <srslib_framework/localization/map/logical/LogicalMap.hpp>

#include <limits>

#include <srslib_framework/datastructure/Location.hpp>
#include <srslib_framework/utils/Filesystem.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap::LogicalMap(double widthM, double heightM, double resolution, Pose<> origin) :
        BaseMap(widthM, heightM, resolution, origin)
{
    metadata_.heightCells = getHeightCells();
    metadata_.heightM = getHeightM();
    metadata_.resolution = getResolution();
    metadata_.widthCells = getWidthCells();
    metadata_.widthM = getWidthM();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap::LogicalMap(Grid2d* grid, double resolution, Pose<> origin) :
    BaseMap(grid, resolution, origin)
{
    metadata_.heightCells = getHeightCells();
    metadata_.heightM = getHeightM();
    metadata_.resolution = getResolution();
    metadata_.widthCells = getWidthCells();
    metadata_.widthM = getWidthM();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap::LogicalMap(LogicalMetadata metadata) :
    BaseMap(metadata.widthM, metadata.heightM, metadata.resolution, metadata.origin)
{
    metadata_.heightCells = getHeightCells();
    metadata_.heightM = getHeightM();
    metadata_.resolution = getResolution();
    metadata_.widthCells = getWidthCells();
    metadata_.widthM = getWidthM();

    metadata_.loadTime = metadata.loadTime;
    metadata_.logicalFilename = metadata.logicalFilename;
    metadata_.origin = metadata.origin;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMap::addLabeledArea(unsigned int ciCells, unsigned int riCells,
    unsigned int cfCells, unsigned int rfCells,
    string label, shared_ptr<MapNotes> notes)
{
    LabeledArea area;
    area.ci = ciCells;
    area.ri = riCells;
    area.cf = cfCells;
    area.rf = rfCells;
    area.label = label;
    area.notes = notes;

    labeledAreas_[label] = area;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMap::checkAreas(unsigned int cCells, unsigned int rCells, LabeledAreaMapType& areas) const
{
    areas.clear();

    for (auto area : labeledAreas_)
    {
        if (cCells >= area.second.ci &&  cCells <= area.second.cf &&
            rCells >= area.second.ri && rCells <= area.second.rf)
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
void LogicalMap::maxCost(unsigned int cCells, unsigned int rCells, Grid2d::BaseType cost)
{
    getGrid()->maxOnPayload(Location(cCells, rCells), cost);
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

    if (lhs.metadata_ == rhs.metadata_ &&
        operator==(static_cast<const BaseMap&>(lhs), static_cast<const BaseMap&>(rhs)))
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
void LogicalMap::setCost(unsigned int cCells, unsigned int rCells, Grid2d::BaseType cost)
{
    getGrid()->setPayload(Location(cCells, rCells), cost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMap::setObstacle(unsigned int cCells, unsigned int rCells)
{
    getGrid()->setPayload(Location(cCells, rCells), Grid2d::PAYLOAD_MAX);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMap::setWeights(unsigned int cCells, unsigned int rCells,
    Grid2d::BaseType north,
    Grid2d::BaseType east,
    Grid2d::BaseType south,
    Grid2d::BaseType west)
{
    getGrid()->setWeights(Location(cCells, rCells), north, east, south, west);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods
//

} // namespace srs

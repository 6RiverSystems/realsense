#include <srslib_framework/ros/message/LogicalMapMessageFactory.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapMessageFactory::areas2Vector(const LogicalMap* map,
    vector<srslib_framework::LogicalArea>& areas)
{
    areas.clear();

    for (auto area : map->getAreas())
    {
        srslib_framework::LogicalArea msgArea;
        msgArea.label = area.second.label;
        msgArea.xi = area.second.xi;
        msgArea.yi = area.second.yi;
        msgArea.xf = area.second.xf;
        msgArea.yf = area.second.yf;
        msgArea.note = note2Msg(area.second.note);

        areas.push_back(msgArea);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapMessageFactory::map2Vector(const LogicalMap* map,
    vector<srslib_framework::LogicalCell>& logical)
{
    logical.clear();

    Grid2d* grid = map->getGrid();
    if (grid)
    {
        for (int row = 0; row < grid->getHeight(); row++)
        {
            for (int col = 0; col < grid->getWidth(); col++)
            {
                srslib_framework::LogicalCell cell;
                Grid2d::Location location = Grid2d::Location(col, row);

                // Transfer the cost
                cell.cost = static_cast<int32_t>(grid->getPayload(location));

                // Transfer the weights
                cell.north = static_cast<int32_t>(grid->getWeight(
                    Grid2d::Position(col, row, Grid2d::ORIENTATION_NORTH)));
                cell.east = static_cast<int32_t>(grid->getWeight(
                    Grid2d::Position(col, row, Grid2d::ORIENTATION_EAST)));
                cell.south = static_cast<int32_t>(grid->getWeight(
                    Grid2d::Position(col, row, Grid2d::ORIENTATION_SOUTH)));
                cell.west = static_cast<int32_t>(grid->getWeight(
                    Grid2d::Position(col, row, Grid2d::ORIENTATION_WEST)));

                logical.push_back(cell);
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
MapNote LogicalMapMessageFactory::msg2Note(srslib_framework::MapNote message)
{
    MapNote note;

    if (message.beep)
    {
        note.join(MapNote::BEEP);
    }

    return note;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
srslib_framework::MapNote LogicalMapMessageFactory::note2Msg(MapNote note)
{
    srslib_framework::MapNote msgNote;

    msgNote.beep = note.beep();

    return msgNote;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapMessageFactory::vector2Areas(const vector<srslib_framework::LogicalArea>& areas,
    LogicalMap* logical)
{
    for (auto area : areas)
    {
        MapNote note = msg2Note(area.note);
        logical->addLabeledArea(area.xi, area.yi, area.xf, area.yf, area.label, note);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap* LogicalMapMessageFactory::vector2Map(const LogicalMetadata& metadata,
    const vector<srslib_framework::LogicalCell>& logical)
{
    LogicalMap* logicalMap = new LogicalMap(metadata);

    auto logicalIterator = logical.begin();
    for (int row = 0; row < metadata.heightCells; row++)
    {
        for (int col = 0; col < metadata.widthCells; col++)
        {
            logicalMap->setCost(col, row, static_cast<int>(logicalIterator->cost));
            logicalMap->setWeights(col, row,
                static_cast<Grid2d::BaseType>(logicalIterator->north),
                static_cast<Grid2d::BaseType>(logicalIterator->east),
                static_cast<Grid2d::BaseType>(logicalIterator->south),
                static_cast<Grid2d::BaseType>(logicalIterator->west));

            logicalIterator++;
        }
    }

    return logicalMap;
}

} // namespace srs

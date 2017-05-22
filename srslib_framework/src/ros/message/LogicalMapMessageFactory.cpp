#include <srslib_framework/ros/message/LogicalMapMessageFactory.hpp>

#include <srslib_framework/datastructure/Location.hpp>
#include <srslib_framework/datastructure/Position.hpp>

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
        msgArea.x0 = area.second.surface.x0;
        msgArea.y0 = area.second.surface.y0;
        msgArea.x1 = area.second.surface.x1;
        msgArea.y1 = area.second.surface.y1;
        msgArea.notes = notes2Msg(area.second.notes);

        areas.push_back(msgArea);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapMessageFactory::map2Vector(const LogicalMap* map,
    vector<srslib_framework::LogicalCell>& logical)
{
    logical.clear();

    WeightedGrid2d* grid = map->getGrid();
    for (size_t row = 0; row < grid->getHeight(); row++)
    {
        for (size_t col = 0; col < grid->getWidth(); col++)
        {
            srslib_framework::LogicalCell cell;
            Location location = Location(col, row);

            // Transfer the cost
            cell.cost = static_cast<int32_t>(grid->getPayload(location));

            // Transfer the weights
            cell.n = static_cast<int32_t>(grid->getWeight(
                Position(col, row, WeightedGrid2d::ORIENTATION_NORTH)));
            cell.ne = static_cast<int32_t>(grid->getWeight(
                Position(col, row, WeightedGrid2d::ORIENTATION_NORTH_EAST)));
            cell.e = static_cast<int32_t>(grid->getWeight(
                Position(col, row, WeightedGrid2d::ORIENTATION_EAST)));
            cell.se = static_cast<int32_t>(grid->getWeight(
                Position(col, row, WeightedGrid2d::ORIENTATION_SOUTH_EAST)));
            cell.s = static_cast<int32_t>(grid->getWeight(
                Position(col, row, WeightedGrid2d::ORIENTATION_SOUTH)));
            cell.sw = static_cast<int32_t>(grid->getWeight(
                Position(col, row, WeightedGrid2d::ORIENTATION_SOUTH_WEST)));
            cell.w = static_cast<int32_t>(grid->getWeight(
                Position(col, row, WeightedGrid2d::ORIENTATION_WEST)));
            cell.nw = static_cast<int32_t>(grid->getWeight(
                Position(col, row, WeightedGrid2d::ORIENTATION_NORTH_WEST)));

            logical.push_back(cell);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
shared_ptr<MapNotes> LogicalMapMessageFactory::msg2Notes(srslib_framework::MapNotes message)
{
    shared_ptr<MapNotes> notes = shared_ptr<MapNotes>(new MapNotes());

    for (auto note : message.notes)
    {
        notes->add(note.note, note.value);
    }

    return notes;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
srslib_framework::MapNotes LogicalMapMessageFactory::notes2Msg(shared_ptr<MapNotes> notes)
{
    srslib_framework::MapNotes msgNotes;

    for (auto note : *notes)
    {
        srslib_framework::MapNote msgSingleNote;
        msgSingleNote.note = note.second->getType();
        msgSingleNote.value = note.second->getValue();

        msgNotes.notes.push_back(msgSingleNote);
    }

    return msgNotes;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapMessageFactory::vector2Areas(const vector<srslib_framework::LogicalArea>& areas,
    LogicalMap* logical)
{
    for (auto area : areas)
    {
        shared_ptr<MapNotes> notes = msg2Notes(area.notes);

        Rectangle surface(area.x0, area.y0, area.x1, area.y1);
        logical->addLabeledArea(surface, area.label, notes);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap* LogicalMapMessageFactory::vector2Map(const LogicalMetadata& metadata,
    const vector<srslib_framework::LogicalCell>& logical)
{
    WeightedGrid2d* grid = new WeightedGrid2d(metadata.widthCells, metadata.heightCells);
    LogicalMap* logicalMap = new LogicalMap(metadata, grid);

    auto logicalIterator = logical.begin();
    for (int row = 0; row < metadata.heightCells; row++)
    {
        for (int col = 0; col < metadata.widthCells; col++)
        {
            logicalMap->costSet(col, row, static_cast<int>(logicalIterator->cost));
            logicalMap->setWeights(col, row,
                static_cast<WeightedGrid2d::BaseType>(logicalIterator->n),
                static_cast<WeightedGrid2d::BaseType>(logicalIterator->ne),
                static_cast<WeightedGrid2d::BaseType>(logicalIterator->e),
                static_cast<WeightedGrid2d::BaseType>(logicalIterator->se),
                static_cast<WeightedGrid2d::BaseType>(logicalIterator->s),
                static_cast<WeightedGrid2d::BaseType>(logicalIterator->sw),
                static_cast<WeightedGrid2d::BaseType>(logicalIterator->w),
                static_cast<WeightedGrid2d::BaseType>(logicalIterator->nw));

            logicalIterator++;
        }
    }

    return logicalMap;
}

} // namespace srs

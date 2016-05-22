#include <srsnode_executive/tap/RosTapMap.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void RosTapMap::onMap(const CompleteMapConstPtr& message)
{
    if (map_)
    {
        delete map_;
    }
    map_ = new Map(message->info.width, message->info.height, message->info.resolution);

    vector<int8_t> costsGrid = message->costs;
    vector<int8_t> notesGrid = message->notes;

    map_->setGrid(costsGrid, notesGrid);
}

} // namespace srs

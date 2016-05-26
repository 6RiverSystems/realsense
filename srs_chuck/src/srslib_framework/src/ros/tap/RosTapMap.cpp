#include <srslib_framework/ros/tap/RosTapMap.hpp>

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
    map_->setGrid(message->costs, message->notes);
}

} // namespace srs
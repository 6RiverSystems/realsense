#include <srslib_framework/localization/map/BaseMap.hpp>

#include <limits>

#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>

#include <srslib_framework/utils/Filesystem.hpp>
#include <srslib_framework/graph/grid2d/Grid2dLocation.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
BaseMap::BaseMap() :
        grid_(nullptr)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
BaseMap::BaseMap(double widthC, double heightC, double resolution) :
        grid_(nullptr)
{
    baseMetadata_.heightCells = heightC;
    baseMetadata_.widthCells = widthC;
    baseMetadata_.resolution = resolution;

    baseMetadata_.widthM = widthC * resolution;
    baseMetadata_.heightM = heightC * resolution;

    grid_ = new Grid2d(widthC, heightC);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
BaseMap::BaseMap(BaseMetadata metadata) :
        grid_(nullptr)
{
    baseMetadata_.heightCells = metadata.heightCells;
    baseMetadata_.widthCells = metadata.widthCells;
    baseMetadata_.resolution = metadata.resolution;

    baseMetadata_.widthM = metadata.widthCells * metadata.resolution;
    baseMetadata_.heightM = metadata.heightCells * metadata.resolution;

    grid_ = new Grid2d(metadata.widthCells, metadata.heightCells);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
BaseMap::~BaseMap()
{
    delete grid_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods
//

////////////////////////////////////////////////////////////////////////////////////////////////////
// Global operators

} // namespace srs

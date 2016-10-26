#include <srslib_framework/localization/map/BaseMap.hpp>

#include <srslib_framework/math/BasicMath.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
BaseMap::BaseMap(unsigned int widthCells, unsigned int heightCells, double resolution) :
        grid_(nullptr),
        resolution_(resolution),
        widthM_(widthCells * resolution),
        heightM_(heightCells * resolution),
        userSpecifiedGrid_(false)
{
    grid_ = new Grid2d(widthCells, heightCells);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
BaseMap::BaseMap(double widthM, double heightM, double resolution) :
        grid_(nullptr),
        resolution_(resolution),
        widthM_(widthM),
        heightM_(heightM),
        userSpecifiedGrid_(false)
{
    unsigned int widthCells = static_cast<unsigned int>(round(widthM / resolution_));
    unsigned int heightCells = static_cast<unsigned int>(round(heightM / resolution_));

    grid_ = new Grid2d(widthCells, heightCells);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
BaseMap::BaseMap(Grid2d* grid, double resolution) :
        resolution_(resolution)
{
    userSpecifiedGrid_ = grid != nullptr;
    grid_ = userSpecifiedGrid_ ? grid : new Grid2d(0, 0);
    widthM_ = grid_->getWidth() * resolution_;
    heightM_ = grid_->getHeight() * resolution_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
BaseMap::~BaseMap()
{
    if (!userSpecifiedGrid_)
    {
        delete grid_;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool operator==(const BaseMap& lhs, const BaseMap& rhs)
{
    if (&lhs == &rhs)
    {
        return true;
    }

    return BasicMath::equal(lhs.resolution_, rhs.resolution_, 0.001) &&
        BasicMath::equal(lhs.widthM_, rhs.widthM_, 0.001) &&
        BasicMath::equal(lhs.heightM_, rhs.heightM_, 0.001) &&
        lhs.userSpecifiedGrid_ == rhs.userSpecifiedGrid_ &&
        *lhs.grid_ == *rhs.grid_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods
//

////////////////////////////////////////////////////////////////////////////////////////////////////
// Global operators

} // namespace srs

#include <srslib_framework/localization/map/BaseMap.hpp>

#include <srslib_framework/math/BasicMath.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
BaseMap::BaseMap(unsigned int widthCells, unsigned int heightCells, double resolution, Pose<> origin) :
        grid_(nullptr),
        resolution_(resolution),
        origin_(origin),
        userSpecifiedGrid_(false)
{
    convertCells2M(widthCells, widthM_);
    convertCells2M(heightCells, heightM_);

    grid_ = new Grid2d(widthCells, heightCells);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
BaseMap::BaseMap(double widthM, double heightM, double resolution, Pose<> origin) :
        grid_(nullptr),
        resolution_(resolution),
        widthM_(widthM),
        heightM_(heightM),
        origin_(origin),
        userSpecifiedGrid_(false)
{
    unsigned int widthCells;
    convertM2Cells(widthM, widthCells);

    unsigned int heightCells;
    convertM2Cells(heightM, heightCells);

    grid_ = new Grid2d(widthCells, heightCells);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
BaseMap::BaseMap(Grid2d* grid, double resolution, Pose<> origin) :
        resolution_(resolution),
        origin_(origin)
{
    userSpecifiedGrid_ = grid != nullptr;
    grid_ = userSpecifiedGrid_ ? grid : new Grid2d(0, 0);

    convertCells2M(grid_->getWidth(), widthM_);
    convertCells2M(grid_->getHeight(), heightM_);
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

#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionItem.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constant definitions

unordered_map<int, string> Grid2dSolutionItem::ENUM_NAMES = {
    {Grid2dSolutionItem::MOVE, "MOVE"},
    {Grid2dSolutionItem::NONE, "NONE"},
    {Grid2dSolutionItem::ROTATE, "ROTATE"},
};

} // namespace srs

#include <srslib_framework/planning/pathplanning/grid/GridSolutionItem.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constant definitions

unordered_map<int, string> GridSolutionItem::ENUM_NAMES = {
    {GridSolutionItem::MOVE, "MOVE"},
    {GridSolutionItem::NONE, "NONE"},
    {GridSolutionItem::ROTATE, "ROTATE"},
};

} // namespace srs

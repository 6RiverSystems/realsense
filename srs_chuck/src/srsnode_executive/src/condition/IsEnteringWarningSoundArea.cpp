#include <srsnode_executive/condition/IsEnteringWarningSoundArea.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
TreeNode<ExecutiveContext>::NodeResult IsEnteringWarningSoundArea::execute(ExecutiveContext* context)
{
    for (auto area : context->activeLabeledAreas)
    {
        if (area.second == ExecutiveContext::DirectionEnum::ENTERING)
        {
            if (area.first.note.warning_sound())
            {
                return SUCCEDED;
            }
        }
    }

    return FAILED;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs

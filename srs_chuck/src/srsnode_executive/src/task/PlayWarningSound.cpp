#include <srsnode_executive/task/PlayWarningSound.hpp>

namespace srs {

const Sound PlayWarningSound::WARNING_SOUND = Sound(100, 3000, 250, 32, 65000);

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
TreeNode<ExecutiveContext>::NodeResult PlayWarningSound::execute(ExecutiveContext* context)
{
    channelSound_.publish(WARNING_SOUND);

    return SUCCEDED;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs

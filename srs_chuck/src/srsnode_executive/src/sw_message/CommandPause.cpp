#include <srsnode_executive/sw_message/CommandPause.hpp>

#include <srsnode_executive/Executive.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
CommandPause::CommandPause(Executive* owner) :
    Command(owner, 1)
{}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool CommandPause::execute(const vector<string>& params)
{
	bool pauseState = param2Bool(params, POSITION_PARAM_1);

	srslib_framework::MsgSetOperationalState operationalState;

	operationalState.operationalState.frontEStop = false;
	operationalState.operationalState.hardStop = false;
	operationalState.operationalState.backEStop = false;
	operationalState.operationalState.wirelessEStop = false;
	operationalState.operationalState.bumpSensor = false;
	operationalState.operationalState.pause = pauseState;

	operationalState.state = true;

    getOwner()->setPauseState(pauseState);


    setMotionState_.publish(operationalState);

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs

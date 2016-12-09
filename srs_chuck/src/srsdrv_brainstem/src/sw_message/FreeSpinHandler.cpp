#include <srsdrv_brainstem/sw_message/FreeSpinHandler.hpp>

#include <srsdrv_brainstem/BrainStemMessageProcessor.h>
#include <srsdrv_brainstem/BrainStemMessages.h>

#include <ros/ros.h>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
FreeSpinHandler::FreeSpinHandler(BrainStemMessageProcessor* owner) :
    SoftwareMessageHandler(owner)
{
    tapFreeSpin_.attach(this);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void FreeSpinHandler::notified(Subscriber<std_msgs::Bool>* subject)
{
    TapBrainstemCmd_FreeSpin* tap = static_cast<TapBrainstemCmd_FreeSpin*>(subject);

    bool freeSpinState = tap->pop();

    ROS_INFO_STREAM_COND_NAMED(freeSpinState, "free_spin_handler", "Robot paused");
    ROS_INFO_STREAM_COND_NAMED(!freeSpinState, "free_spin_handler", "Robot unpaused");

    std::bitset<8> freeSpinSet;
    freeSpinSet.set(MOTION_STATUS::FREE_SPIN, true);

    getOwner()->setMotionStatus(freeSpinSet, freeSpinState);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods


} // namespace srs

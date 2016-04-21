#include "RosBrainStemStatus.hpp"

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
bool RosBrainStemStatus::connect()
{
    rosSubscriber_ = rosNodeHandle_.subscribe("/brain_stem/connected", 100,
        &RosBrainStemStatus::onBrainStemConnected, this);

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void RosBrainStemStatus::onBrainStemConnected(std_msgs::BoolConstPtr message)
{
    isBrainStemConnected_ = message->data;
    connectionStateChanged_ = prevBrainStemConnected_ != isBrainStemConnected_;

    if (connectionStateChanged_)
    {
        ROS_INFO_STREAM("Brain stem changed the connection state from " <<
            prevBrainStemConnected_ << " to " << isBrainStemConnected_);
    }

    prevBrainStemConnected_ = isBrainStemConnected_;

    ros::Time timestamp = ros::Time::now();
    setNewData(true, timestamp.nsec);
}

} // namespace srs

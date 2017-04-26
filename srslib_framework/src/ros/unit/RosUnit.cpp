namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
template<class T>
RosUnit<T>::RosUnit(string name, int argc, char** argv, double refreshRate) :
    BaseRos(name, argc, argv),
    rate_(refreshRate),
    refreshRateFrequency_(refreshRate),
    previousTime_(ros::Time::now()),
    currentTime_(ros::Time::now()),
    rosNodeHandle_(name)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<class T>
RosUnit<T>::~RosUnit()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<class T>
void RosUnit<T>::run()
{
    initialize();

    while (ros::ok())
    {
        ros::spinOnce();

        previousTime_ = currentTime_;
        currentTime_ = ros::Time::now();

        execute();

        rate_.sleep();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<class T>
void RosUnit<T>::getGlobalParameter(string service, string parameter,
    float& value, float defaultValue)
{
    // TODO Make this method more intelligent (i.e. detect additional "/", etc)
    rosNodeHandle_.param(service + "/" + parameter, value, defaultValue);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<class T>
void RosUnit<T>::getLocalParameter(string parameter, string& value, string defaultValue)
{
    rosNodeHandle_.param(parameter, value, defaultValue);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs

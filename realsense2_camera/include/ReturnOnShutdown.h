#pragma once
#ifndef SRS_RETURN_ON_SHUTDOWN_H
#define SRS_RETURN_ON_SHUTDOWN_H

#define RETURN_ON_SHUTDOWN()                                         \
{                                                                    \
    if(ros::isShuttingDown())                                        \
    {                                                                \
        ROS_ERROR_STREAM("ROS is shutting down returning early");    \
        return;                                                      \
    }                                                                \
}

#define REQUEST_SHUTDOWN()                                           \
{                                                                    \
    RETURN_ON_SHUTDOWN();                                            \
    ros::requestShutdown();                                          \
    ros::shutdown();                                                 \
    sleep(5);                                                        \
    return;                                                          \
}
#endif
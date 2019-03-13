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
    ROS_ERROR_STREAM("Requesting USB bus reset");                    \
    std_msgs::Bool msg;                                              \
    msg.data = true;                                                 \
    _reset_request_publisher.publish(msg);                           \
    sleep(1);                                                        \
    ros::requestShutdown();                                          \
    sleep(1);                                                        \
    ros::shutdown();                                                 \
    sleep(3);                                                        \
    return;                                                          \
}
#endif
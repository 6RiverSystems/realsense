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
    std_msgs::String msg;                                            \
    msg.data = _sensor_name + _usb_port_id;                          \
    _reset_request_publisher.publish(msg);                           \
    srslib_framework::Event data;                                    \
    data.entity = _sensor_entity;                                    \
    data.value = _sensor_value;                                      \
    data.attribute = _sensor_name + _usb_port_id + ".USB_BUS_RESET_REQUEST_SENT"; \
    sleep(1);                                                        \
    ros::requestShutdown();                                          \
    sleep(1);                                                        \
    ros::shutdown();                                                 \
    sleep(3);                                                        \
    return;                                                          \
}
#endif
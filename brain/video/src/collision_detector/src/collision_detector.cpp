#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/String.h>

#include <iostream>
#include <boost/thread.hpp>
#include <thread>
#include <cstdlib>
#include <cctype>
#include <algorithm>
#include <sstream>
#include <memory>
#include <map>

#include <librealsense/rs.hpp>

enum class REALSENSE_STATE
{
    UNKNOWN,
    DISCONNECTED,
    CONNECTED
};

REALSENSE_STATE g_eRealSenseState = REALSENSE_STATE::UNKNOWN;

// TODO: Deterimine coding style guidelines
// TODO: Implement linting for unit tests
// TODO: Implement gtest/test cases/circleci integration
// TODO: Create class for detector
// TODO: Use boost bind to move this into a class and class member...
void setObjectDetected( ros::Publisher& publisher, bool bObjectDetected )
{
    static bool sbObjectDetected = false;

    if( sbObjectDetected != bObjectDetected )
    {
        sbObjectDetected = bObjectDetected;

        std::string strMessage = "OBSTACLE";

        // Publish message
        if( sbObjectDetected )
        {
            printf("Object detected\n");

            strMessage += " T";
        }
        else
        {
            printf("Object removed\n");

            strMessage += " F";
        }


        std_msgs::String msg;

        msg.data = strMessage;

        ROS_INFO("%s", msg.data.c_str());

        publisher.publish(msg);
    }
}

void SetRealSenseState( REALSENSE_STATE eRealSenseState )
{
    if( g_eRealSenseState != eRealSenseState )
    {
        switch( eRealSenseState )
        {
            case REALSENSE_STATE::UNKNOWN:
            {
            }
            break;

            case REALSENSE_STATE::DISCONNECTED:
            {
                if( g_eRealSenseState == REALSENSE_STATE::UNKNOWN )
                {
                   printf("Realsense camera not found, will check every 1 second.\n");
                }
                else
                {
                    printf("Realsense camera disconnected\n");
                }
            }
            break;

            case REALSENSE_STATE::CONNECTED:
            {
                printf("Realsense camera connected\n");
            }
            break;
        }

        g_eRealSenseState = eRealSenseState;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_detector");

    ros::NodeHandle n;

    ros::Publisher cmd_ll = n.advertise<std_msgs::String>("/ll_event", 1000);

    ros::Rate loop_rate(10);

    bool bKeepRunning = true;

    while( bKeepRunning )
    {
        // Create a context object. This object owns the handles to all connected realsense devices.
        rs::context ctx;

        if( ctx.get_device_count() != 0 )
        {
            try
            {
                SetRealSenseState( REALSENSE_STATE::CONNECTED );

                // This tutorial will access only a single device, but it is trivial to extend to multiple devices
                rs::device * dev = ctx.get_device(0);
                printf("\nUsing device 0, an %s\n", dev->get_name());
                printf("    Serial number: %s\n", dev->get_serial());
                printf("    Firmware version: %s\n", dev->get_firmware_version());

                // Configure depth to run at VGA resolution at 30 frames per second
                dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 60);
                dev->start();

                // Determine depth value corresponding to one meter
                const uint16_t one_meter = static_cast<uint16_t>(.3f / dev->get_depth_scale());

                while( ros::ok( ) )
                {
                    // This call waits until a new coherent set of frames is available on a device
                    // Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
                    dev->wait_for_frames();

                    // Retrieve depth data, which was previously configured as a 640 x 480 image of 16-bit depth values
                    const uint16_t * depth_frame = reinterpret_cast<const uint16_t *>(dev->get_frame_data(rs::stream::depth));

                    bool bObjectDetected = false;

                    // Print a simple text-based representation of the image, by breaking it into 10x20 pixel regions and and approximating the coverage of pixels within one meter
                    char buffer[(640/10+1)*(480/20)+1];
                    char * out = buffer;
                    int coverage[64] = {};
                    for(int y=0; y<480; ++y)
                    {
                        for(int x=0; x<640; ++x)
                        {
                            int depth = *depth_frame++;
                            if(depth > 0 && depth < one_meter)
                            {
                                ++coverage[x/10];
                            }
                        }

                        if(y%20 == 19)
                        {
                            for(int & c : coverage)
                            {
                                if(c/25 > 2)
                                {
                                    bObjectDetected = true;
                                }
                                *out++ = " .:nhBXWW"[c/25];
                                c = 0;
                            }
                            *out++ = '\n';
                        }
                    }
                    *out++ = 0;

//                    printf("\n%s", buffer);

                    setObjectDetected( cmd_ll, bObjectDetected );

                    // run ros event loop
                    ros::spinOnce();

                    loop_rate.sleep();
                }

                bKeepRunning = false;
            }
            catch(...)
            {
                // Device disconnected or some other error, wait and retry
                SetRealSenseState( REALSENSE_STATE::DISCONNECTED );
            }
        }
        else
        {
            SetRealSenseState( REALSENSE_STATE::DISCONNECTED );

            sleep(1);
        }
    }

    return EXIT_SUCCESS;
}

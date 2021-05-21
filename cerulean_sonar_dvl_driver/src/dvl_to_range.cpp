
/*
 * DVL to Range
 * Purpose: Converts a DVL message to a Range message, so it can be used for altitude control.
 * Subscribes:
 *  - 'dvl_extended_data': The dvl message
 * Publishes:
 * - 'altitude': The altitude
 * Services:
 * -
 * Outputs: Altitude
*/
#include <math.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include <cerulean_sonar_dvl_msgs/DVLExtendedData.h>

class DvlToRange
{
public:
    DvlToRange()
    {
        altPub_ = nh_.advertise<sensor_msgs::Range>("dvl_altitude", 1);

        dvlSub_ =  nh_.subscribe("dvl_extended_data", 1, &DvlToRange::dvlCallback, this);
        
    }


    // Receives the sonar altitude
    void dvlCallback(const sensor_msgs::Range::ConstPtr& msg)
    {
        sensor_msgs::Range range_msg;
        range_msg.header = msg->header;
        range_msg.altitude = msg->altitude;
        range_msg.max_range = 40.0;
        range_msg.min_range = 0.3;
        range_msg.field_of_view =0.523599; // 30 degrees 
        altPub_.publish(range_msg);
    }
    

private:
    // The nodehandle
    ros::NodeHandle nh_;

    // Publishers
    ros::Publisher altPub_;

    // Subscribers
    ros::Subscriber dvlSub_;

};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "altitude_select");

    AltitudeSelect selector;

    ros::spin();

    return 0;
}

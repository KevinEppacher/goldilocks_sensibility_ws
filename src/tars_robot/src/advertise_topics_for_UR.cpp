#include "ros/ros.h"
#include <sstream>
#include "Measurement.h"


int main(int argc, char **argv)
{

    ros::init(argc, argv, "advertising_topics");
    ros::NodeHandle nh;
    Measurement::Sensibility ur10(nh);
    ros::Rate loop_rate(10);      
    
    while (ros::ok())
    {   
        ur10.publishAbsoluteForce();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
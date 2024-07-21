#include "ros/ros.h"
#include <sstream>
#include "Measurement.h"
#include "get_pose.h"


int main(int argc, char **argv)
{

    ros::init(argc, argv, "teaching_mode");
    ros::NodeHandle nh;
    Measurement::Sensibility ur10(&nh);

    ros::Rate loop_rate(10); 

    ROS_INFO("Stop Teaching Mode");     
    
    while (ros::ok())
    {   
        ur10.stopTeachingMode();
        // Pose pose(nh, "table_link", "base_link");
        // pose.getPose();
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include "Measurement.h"

////////////////////////////////////////////////////
// Pseudo code:
// setup:
//     Store all published pose arrays in a vector
//     visualization
//     manipulator config
//     setup measurement (filepath, filename, etc.)

// loop:
//     for each pose in the vector:
//         turn camera on
//         go to prePose(-z direction 100mm) 
//         take a picture
//         go to the pose
//         turn camera off
//         make measurement
//             move in z until AIRSKIN is triggered
//                 store the force
//                 store the path lenght
//                 if path lenght > maxPathLenght:
//                     break
////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    ros::init(argc, argv, "start_sensibility_measurements");
    ros::NodeHandle nh;
    Measurement::Sensibility sens(nh);
    sens.run_measurement();
    ros::shutdown();
    return 0;
}
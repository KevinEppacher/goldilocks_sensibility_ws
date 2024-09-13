

// Libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt32.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64.h"
#include <fstream>
#include <ros/time.h>
#include <sstream>
#include <fstream>
#include <string>
#include <iomanip>
#include <ctime>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <iostream>
#include <chrono>
#include <sys/stat.h>
#include <std_msgs/Float64.h>

// Custom classes
#include "Robot.h"

namespace Measurement
{

    class Sensibility
    {
    public:
        Sensibility(ros::NodeHandle &nodehandler);
        ~Sensibility();

        // Publisher
        void publishAbsoluteForce();

        // Subscribers
        void forceTorqueSensorCallback(const geometry_msgs::Pose::ConstPtr &forceTorque);
        void poseUrCallback(const geometry_msgs::Pose::ConstPtr &poseUR);
        void measurementPointsCallback(const geometry_msgs::PoseArray::ConstPtr &measurementsPointsMsg);

        // Sensibility measurement
        void run_measurement();

    private:
        // Utility functions
        void loadParameters();
        std::string getCurrentDateTime();
        void createDirectory(const std::string &dirName);

        // Sensibility measurement version 1
        std_msgs::Float64 msgAbsoluteForce;
        geometry_msgs::Pose currentPose, lastPose, distanceVektor;
        std_msgs::Float64 msgDistance;
        std_msgs::Float64 scalarDistance;

        // ROS variables
        ros::NodeHandle nh;
        ros::Publisher forceTorqueSensor;
        ros::Publisher absoluteForce;
        ros::Subscriber forceTorqueSensorSub;
        ros::Publisher poseUR;
        ros::Subscriber poseUrSub;
        ros::Subscriber measurementPointsSub;

        // Custom classes
        // Robot::ArticulatedRobot ur10;
        
        // Sensibility measurement version 2
        geometry_msgs::PoseArray poses;

        // General
        std::string nodeName = "Sensibility";

        // Measurement-related variables
        double max_measuring_distance;
        double linearVelocity, linearAcceleration;
        double ptpVelocity, ptpAcceleration;

        bool startMeasurement = false;
        int measurementID = 0;

        // File and folder variables
        std::string mainFolder;
        std::string poseFolder;
        std::string csvFilePath;
        std::ofstream csvFile;
    };

}



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

// Custom classes
#include "Robot.h"

namespace Measurement
{

    class Sensibility
    {
    public:
        Sensibility(ros::NodeHandle &nodehandler);
        ~Sensibility();
        // Publishers
        void publishAbsoluteForce();

        // Subscibers
        void forceTorqueSensorCallback(const geometry_msgs::Pose::ConstPtr &forceTorque);
        void poseUrCallback(const geometry_msgs::Pose::ConstPtr &poseUR);
        void measurementPointsCallback(const geometry_msgs::PoseArray::ConstPtr &measurementsPointsMsg);

        // Sensibility measurement
        void run_measurement();

    private:
        void loadParameters();

        // Sensibility measurement version 1
        std_msgs::Float64 msgAbsoluteForce;
        geometry_msgs::Pose currentPose, lastPose, distanceVektor;
        std_msgs::Float64 msgDistance;
        std_msgs::Float64 scalarDistance;
        ros::NodeHandle nh;
        ros::Publisher forceTorqueSensor;
        ros::Publisher absoluteForce;
        ros::Subscriber forceTorqueSensorSub;
        ros::Publisher poseUR;
        ros::Subscriber poseUrSub;

        // Sensibility measurement version 2
        ros::Subscriber measurementPointsSub;
        geometry_msgs::PoseArray poses;

        // general
        std::string nodeName = "Sensibility";

        double maxDistance;
    };

    class csvPlotter
    {
    public:
        csvPlotter(const std::string &filename);
        csvPlotter();
        ~csvPlotter();
        bool createCSVFile();
        void getTime();
        bool createExcelFile();
        bool writeData(std_msgs::Float64 msgAbsoluteForce, std_msgs::Int16 airskinPadNumber, std_msgs::String airskinState);
        bool writeCSVData(std_msgs::Float64 *msgAbsoluteForce, std_msgs::Int16 *airskinPadNumber, std_msgs::String *airskinState, std_msgs::Float64 *distance);
        bool writeExcelData(std_msgs::Float64 *msgAbsoluteForce, std_msgs::Int16 *airskinPadNumber, std_msgs::String *airskinState, std_msgs::Float64 *distance);
        bool closeWorkbook();

    private:
        std::string filename;
        std::ofstream sensitivityFile;
        std::time_t currentTime;
        std::tm *localTime;
        std::uint32_t currentTimeSec;
        float time = 0, frequency = 0.001;
        int rowArray[15];
        int row = 1;
        std::string msgAbsoluteForceString;
        std::string airskinPadNumberString;
        std::string timeString;
        std::string airskinStateString;
        std::string distanceString;
        int airskinPadNumberInt;
        std::ofstream file;
        int PadQuantity = 15;
        int sectionColumn = 4;
        int colSection = 4;
        int columThreshold;
        int quantityOfZeros = 0;
    };

}

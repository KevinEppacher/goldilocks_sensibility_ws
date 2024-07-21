
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64.h"
#include <fstream>
#include <ros/time.h>
#include <sstream>
#include <fstream>
#include <string>
#include <iomanip>
#include <ctime>


namespace Measurement {

    class Sensibility
    {
    public:
        Sensibility(ros::NodeHandle *nh);
        ~Sensibility();
        void publishAbsoluteForce();
        void forceTorqueSensorCallback(const geometry_msgs::Pose::ConstPtr& forceTorque);
        void airskinPadNumCallback(const std_msgs::Int16::ConstPtr& airskinPadNumber);
        void airskinInterruptCallback(const std_msgs::String::ConstPtr& airskinState);
        void poseUrCallback(const geometry_msgs::Pose::ConstPtr& poseUR);
        void stopTeachingMode();
        std_msgs::String msgAirskinState;
        std_msgs::Float64 msgAbsoluteForce;
        std_msgs::Int16 msgAirskinPadNum;     
        geometry_msgs::Pose currentPose, lastPose, distanceVektor;
        std_msgs::Float64 msgDistance;
        std_msgs::Float64 scalarDistance;
        std_msgs::Int16 msgStopTeachingMode;

    private:
        ros::NodeHandle *nodeHandler;
        ros::Subscriber absoluteForceSub;
        ros::Publisher forceTorqueSensor;
        ros::Publisher absoluteForce;
        ros::Publisher airskinInterrupt;
        ros::Publisher airskinPadNum;
        ros::Publisher maxAirskinPads;
        ros::Subscriber forceTorqueSensorSub;
        ros::Subscriber airskinPadNumSub;
        ros::Subscriber airskinInterruptSub;
        ros::Publisher poseUR;
        ros::Subscriber poseUrSub;
        ros::Publisher stopTeachingModePub;

    };

    class csvPlotter
    {
    public:
        csvPlotter(const std::string& filename);
        csvPlotter();
        ~csvPlotter();
        bool createCSVFile();
        void getTime();
        bool createExcelFile();
        bool writeData(std_msgs::Float64 msgAbsoluteForce, std_msgs::Int16 airskinPadNumber, std_msgs::String airskinState);
        bool writeCSVData(std_msgs::Float64* msgAbsoluteForce, std_msgs::Int16* airskinPadNumber, std_msgs::String* airskinState, std_msgs::Float64* distance);
        bool writeExcelData(std_msgs::Float64* msgAbsoluteForce, std_msgs::Int16* airskinPadNumber, std_msgs::String* airskinState, std_msgs::Float64* distance);
        bool closeWorkbook();


    private:
        std::string filename;
        std::ofstream sensitivityFile;
        std::time_t currentTime;
        std::tm* localTime ;
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

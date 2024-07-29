#include "Measurement.h"


Measurement::Sensibility::Sensibility(ros::NodeHandle *nh)
{
    forceTorqueSensor = nh->advertise<geometry_msgs::Pose>("force_torque_sensor", 100);
    absoluteForce = nh->advertise<std_msgs::Float64>("absolute_force", 100);
    airskinInterrupt = nh->advertise<std_msgs::String>("AIRSKIN_Interrupt", 100);
    airskinPadNum = nh->advertise<std_msgs::Int16>("AIRSKIN_Pad_Number", 100);
    maxAirskinPads = nh->advertise<std_msgs::Int16>("Maximum_Airskin_Pads", 100);
    stopTeachingModePub = nh->advertise<std_msgs::UInt32>("teaching_mode", 1);
    forceTorqueSensorSub = nh->subscribe("force_torque_sensor", 100, &Measurement::Sensibility::forceTorqueSensorCallback, this);
    airskinPadNumSub = nh->subscribe("AIRSKIN_Pad_Number", 100, &Measurement::Sensibility::airskinPadNumCallback, this);
    airskinInterruptSub = nh->subscribe("AIRSKIN_Interrupt", 100, &Measurement::Sensibility::airskinInterruptCallback, this);
    poseUR = nh->advertise<geometry_msgs::Pose>("pose_ur_robot", 100);
    poseUrSub = nh->subscribe("pose_ur_robot", 100, &Measurement::Sensibility::poseUrCallback, this);
}

Measurement::Sensibility::~Sensibility()
{

}

void Measurement::Sensibility::stopTeachingMode()
{
    msgStopTeachingMode.data = 0;
    ROS_INFO("Teaching Mode stopped %d", msgStopTeachingMode.data);
    stopTeachingModePub.publish(msgStopTeachingMode);
    ROS_INFO("Teaching Mode stopped");
}

void Measurement::Sensibility::forceTorqueSensorCallback(const geometry_msgs::Pose::ConstPtr& forceTorque)
{
    msgAbsoluteForce.data = sqrt( pow(forceTorque->position.x , 2) + pow(forceTorque->position.y , 2) + pow(forceTorque->position.z , 2));
    //absoluteForce.publish(msgAbsoluteForce);
}

void Measurement::Sensibility::publishAbsoluteForce()
{
    absoluteForce.publish(msgAbsoluteForce);
}

void Measurement::Sensibility::airskinPadNumCallback(const std_msgs::Int16::ConstPtr& airskinPadNumber)
{
    msgAirskinPadNum.data = airskinPadNumber->data;
}

void Measurement::Sensibility::airskinInterruptCallback(const std_msgs::String::ConstPtr& airskinState)
{
    msgAirskinState.data = airskinState->data;
}

void Measurement::Sensibility::poseUrCallback(const geometry_msgs::Pose::ConstPtr& poseUR)
{
    currentPose = *poseUR;

    if (poseUR->orientation.w == 1)
    {
        distanceVektor.position.x += abs(currentPose.position.x - lastPose.position.x);
        distanceVektor.position.y += abs(currentPose.position.y - lastPose.position.y);
        distanceVektor.position.z += abs(currentPose.position.z - lastPose.position.z);

        scalarDistance.data = sqrt( pow(distanceVektor.position.x , 2) + pow(distanceVektor.position.y , 2) + pow(distanceVektor.position.z , 2));
    }
    
    lastPose = currentPose;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Measurement::csvPlotter::csvPlotter(const std::string& filename) : filename(filename)
{
    
}

Measurement::csvPlotter::csvPlotter()
{

}

Measurement::csvPlotter::~csvPlotter()
{
    
}

void Measurement::csvPlotter::getTime()
{
    currentTime = std::time(nullptr);
    localTime = std::localtime(&currentTime);
}

bool Measurement::csvPlotter::createCSVFile()
{
    Measurement::csvPlotter::getTime();

    std::stringstream dateStream;
    dateStream << std::setw(4) << std::setfill('0') << (localTime->tm_year + 1900) << "-"
               << std::setw(2) << std::setfill('0') << (localTime->tm_mon + 1) << "-"
               << std::setw(2) << std::setfill('0') << localTime->tm_mday << "-"
               << std::setw(2) << std::setfill('0') << localTime->tm_hour << "-"
               << std::setw(2) << std::setfill('0') << localTime->tm_min;

    std::string formattedDate = dateStream.str();

    std::cout << formattedDate << std::endl;

    filename = "src/sensibility_measurements/measurements/" + formattedDate + "-Sensitivity-Measurement.csv";

    file.open(filename, std::ios::app);

    if (!file.is_open()) 
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return 0;
    }

    file << "Time [s],";  
    for(int i = 1; i <= PadQuantity; i++)
    {
        file << "Pad-Nr "<<i<<",Absolute Force [N],Airskin State, Distance [mm],";
    }
    file<<",\n";
    

    return 1;
}

bool Measurement::csvPlotter::writeCSVData(std_msgs::Float64* msgAbsoluteForce, std_msgs::Int16* airskinPadNumber, std_msgs::String* airskinState, std_msgs::Float64* distance)
{
    Measurement::csvPlotter::getTime();

    msgAbsoluteForceString = std::to_string(msgAbsoluteForce->data);
    airskinPadNumberString = std::to_string(airskinPadNumber->data);
    timeString= std::to_string(time);
    distanceString = std::to_string(distance->data);
    airskinPadNumberInt = airskinPadNumber->data;

    file<<timeString<<",";

    if (airskinPadNumberInt != 0)
    {
        if(airskinState->data == "True")
        {
            airskinStateString = "1";
        }
        else
        {
            airskinStateString = "0";
        }

        quantityOfZeros = sectionColumn * airskinPadNumber->data - sectionColumn;

        for (size_t i = 0; i < quantityOfZeros; i++)
        {
            file<<"0,";
        }
        file<<airskinPadNumberString<<","<<msgAbsoluteForceString<<","<<airskinStateString<<","<<distanceString;
        file<<"\n";
        
        std::cout<<"Time: "<< timeString.c_str() <<"   || AIRSKIN Pad Nummer: "<<airskinPadNumberString.c_str()<<" || Absolute Force: "<<msgAbsoluteForceString.c_str()<<"  || AIRSKIN State: "<< airskinStateString.c_str() <<"   || distance: "<< distanceString.c_str() <<std::endl;
    }
    
    return 1;
}



#include "Measurement.h"

namespace Measurement
{
    Sensibility::Sensibility(ros::NodeHandle& nodehandler) : nh(nodehandler)
    {
        // Publisher and Subscriber 
        forceTorqueSensor = nh.advertise<geometry_msgs::Pose>("force_torque_sensor", 100);
        absoluteForce = nh.advertise<std_msgs::Float64>("absolute_force", 100);
        forceTorqueSensorSub = nh.subscribe("force_torque_sensor", 100, &Sensibility::forceTorqueSensorCallback, this);
        poseUR = nh.advertise<geometry_msgs::Pose>("pose_ur_robot", 100);
        poseUrSub = nh.subscribe("pose_ur_robot", 100, &Sensibility::poseUrCallback, this);

        // Publisher and Subscriber for sensibility measurement
        measurementPointsSub = nh.subscribe("measurement_points", 1, &Sensibility::measurementPointsCallback, this);
        
    }

    Sensibility::~Sensibility()
    {

    }

    void Sensibility::forceTorqueSensorCallback(const geometry_msgs::Pose::ConstPtr& forceTorque)
    {
        msgAbsoluteForce.data = sqrt( pow(forceTorque->position.x , 2) + pow(forceTorque->position.y , 2) + pow(forceTorque->position.z , 2));
    }

    void Sensibility::publishAbsoluteForce()
    {
        absoluteForce.publish(msgAbsoluteForce);
    }

    void Sensibility::poseUrCallback(const geometry_msgs::Pose::ConstPtr& poseUR)
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

    void Sensibility::measurementPointsCallback(const geometry_msgs::PoseArray::ConstPtr& measurementsPointsMsg)
    {
        poses = *measurementsPointsMsg;
        // std::cout<<"Number of poses: "<<poses.poses.size()<<std::endl;
        // for(auto& pose : poses.poses)
        // {
        //     ROS_INFO_NAMED("Sensibility", "Pose: x=%f, y=%f, z=%f", pose.position.x, pose.position.y, pose.position.z);
        // }
    }

    void Sensibility::run_measurement()
    {
        Robot::ArticulatedRobot ur10;

        ur10.LIN("tool0_link", 0.1, 0.1);

        // for(auto& pose : poses.poses)
        // {
        //     ur10.PTP( pose );
        // }        
    }



    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    csvPlotter::csvPlotter(const std::string& filename) : filename(filename)
    {
        
    }

    csvPlotter::csvPlotter()
    {

    }

    csvPlotter::~csvPlotter()
    {
        
    }

    void csvPlotter::getTime()
    {
        currentTime = std::time(nullptr);
        localTime = std::localtime(&currentTime);
    }

    bool csvPlotter::createCSVFile()
    {
        csvPlotter::getTime();

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

    bool csvPlotter::writeCSVData(std_msgs::Float64* msgAbsoluteForce, std_msgs::Int16* airskinPadNumber, std_msgs::String* airskinState, std_msgs::Float64* distance)
    {
        csvPlotter::getTime();

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
}



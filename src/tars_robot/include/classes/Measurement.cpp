#include "Measurement.h"

namespace Measurement
{
    Sensibility::Sensibility(ros::NodeHandle &nodehandler) : nh(nodehandler)
    {
        // Publisher and Subscriber
        forceTorqueSensor = nh.advertise<geometry_msgs::Pose>("force_torque_sensor", 100);
        absoluteForce = nh.advertise<std_msgs::Float64>("absolute_force", 100);
        forceTorqueSensorSub = nh.subscribe("force_torque_sensor", 100, &Sensibility::forceTorqueSensorCallback, this);
        poseUR = nh.advertise<geometry_msgs::Pose>("pose_ur_robot", 100);
        poseUrSub = nh.subscribe("pose_ur_robot", 100, &Sensibility::poseUrCallback, this);

        // Publisher and Subscriber for sensibility measurement
        measurementPointsSub = nh.subscribe("measurement_points", 1, &Sensibility::measurementPointsCallback, this);

        loadParameters();
    }

    Sensibility::~Sensibility()
    {
        if (csvFile.is_open())
        {
            csvFile.close();
        }
    }

    void Sensibility::loadParameters()
    {
        nh.param("robot_motion/max_measuring_distance", max_measuring_distance, 0.05);
        nh.param("robot_motion/linear_velocity", linearVelocity, 0.5);
        nh.param("robot_motion/linear_acceleration", linearAcceleration, 0.5);
        nh.param("robot_motion/ptp_velocity", ptpVelocity, 0.05);
        nh.param("robot_motion/ptp_acceleration", ptpAcceleration, 0.05);
    }

    void Sensibility::publishAbsoluteForce()
    {
        absoluteForce.publish(msgAbsoluteForce);
    }

    std::string Sensibility::getCurrentDateTime()
    {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M");
        return ss.str();
    }

    void Sensibility::createDirectory(const std::string &dirName)
    {
        if (mkdir(dirName.c_str(), 0777) == -1)
        {
            ROS_ERROR_STREAM("Error creating directory: " << strerror(errno));
        }
        else
        {
            ROS_INFO_STREAM("Directory created: " << dirName);
        }
    }

    void Sensibility::forceTorqueSensorCallback(const geometry_msgs::Pose::ConstPtr &forceTorque)
    {
        // Berechne die absolute Kraft
        msgAbsoluteForce.data = sqrt(pow(forceTorque->position.x, 2) + pow(forceTorque->position.y, 2) + pow(forceTorque->position.z, 2));

        if (startMeasurement)
        {
            // Hole die aktuelle Pose des Endeffektors von ArticulatedRobot
            // Robot::ArticulatedRobot robot(nh);
            // geometry_msgs::Pose currentPose = robot.getCurrentPose("tool0_link");

            // Berechne die Distanz zwischen der aktuellen und der letzten Pose
            double deltaX = currentPose.position.x - lastPose.position.x;
            double deltaY = currentPose.position.y - lastPose.position.y;
            double deltaZ = currentPose.position.z - lastPose.position.z;

            double deltaDistance = sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);

            // Summiere die Distanz über die Zeit
            scalarDistance.data += deltaDistance;

            ROS_INFO_NAMED("Sensibility", "Current distance: %f", scalarDistance.data);

            // Schreibe die Daten in die CSV-Datei
            if (csvFile.is_open())
            {
                csvFile << ros::Time::now() << "," << msgAbsoluteForce.data << "," << scalarDistance.data * 1000 << "\n";  // Distanz in mm
            }

            // Speichere die aktuelle Pose als letzte Pose für die nächste Iteration
            lastPose = currentPose;
        }
    }


    void Sensibility::poseUrCallback(const geometry_msgs::Pose::ConstPtr &poseUR)
    {
        currentPose = *poseUR;
    }

    void Sensibility::measurementPointsCallback(const geometry_msgs::PoseArray::ConstPtr &measurementsPointsMsg)
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
        bool withActiveAirskin = true;

        Robot::ArticulatedRobot ur10(nh);

        ROS_INFO("Starting the sensibility measurement...");

        if (poses.poses.size() == 0)
        {
            ROS_ERROR("No poses to measure");
            return;
        }

        // Hole den Pfad zum tars_robot-Package
        std::string packagePath = ros::package::getPath("tars_robot");
        if (packagePath.empty())
        {
            ROS_ERROR("Could not find package path for tars_robot.");
            return;
        }

        // Speicherpfad im data-Verzeichnis des tars_robot-Packages
        std::string dataPath = packagePath + "/data";
        ROS_INFO_STREAM("Data will be saved to: " << dataPath);

        // Erstelle den Hauptordner für die Messungen
        mainFolder = dataPath + "/" + getCurrentDateTime() + "_Measurement-Set";
        createDirectory(mainFolder);
        ROS_INFO_STREAM("Created main measurement directory: " << mainFolder);

        measurementID = 0;

        for (auto &pose : poses.poses)
        {
            measurementID++;
            poseFolder = mainFolder + "/" + std::to_string(measurementID) + "_Measurement";
            createDirectory(poseFolder);
            ROS_INFO_STREAM("Created folder for measurement " << measurementID << ": " << poseFolder);

            // Erstelle die CSV-Datei
            csvFilePath = poseFolder + "/" + std::to_string(measurementID) + "_Measurement.csv";
            csvFile.open(csvFilePath);
            if (!csvFile.is_open())
            {
                ROS_ERROR_STREAM("Could not create CSV file: " << csvFilePath);
                return;
            }
            csvFile << "Timestamp,AbsoluteForce,Distance(mm)\n";  // CSV-Header
            ROS_INFO_STREAM("CSV file created: " << csvFilePath);

            ur10.PTP(pose, ptpVelocity, ptpAcceleration);
            startMeasurement = true;

            // LIN fahren (Messung)
            ur10.LIN("tool0_link", max_measuring_distance, linearVelocity, linearAcceleration);

            // Nach der Messung die Messung stoppen
            startMeasurement = false;

            csvFile.close();
            ROS_INFO_STREAM("Measurement " << measurementID << " completed and saved to CSV at: " << csvFilePath);

            // Zurückfahren
            ros::Duration(2.0).sleep();
            ur10.LIN("tool0_link", -max_measuring_distance, linearVelocity * 10, linearAcceleration * 10, withActiveAirskin);
        }
    }
}

#include "Measurement.h"

namespace Measurement
{
    Sensibility::Sensibility(ros::NodeHandle &nodehandler) : nh(nodehandler), tfListener(tfBuffer)
    {
        // Publisher und Subscriber
        forceTorqueSensor = nh.advertise<geometry_msgs::Pose>("force_torque_sensor", 100);
        absoluteForce = nh.advertise<std_msgs::Float64>("absolute_force", 100);
        forceTorqueSensorSub = nh.subscribe("force_torque_sensor", 100, &Sensibility::forceTorqueSensorCallback, this);
        poseUR = nh.advertise<geometry_msgs::Pose>("pose_ur_robot", 100);
        poseUrSub = nh.subscribe("pose_ur_robot", 100, &Sensibility::poseUrCallback, this);
        markerPub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

        // Publisher und Subscriber für Sensibility-Messungen
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
            // Hole die aktuelle Pose des Endeffektors
            currentPose = getCurrentTCPPose();

            // Berechne die Distanz zwischen der aktuellen und der letzten Pose
            double deltaX = currentPose.position.x - lastPose.position.x;
            double deltaY = currentPose.position.y - lastPose.position.y;
            double deltaZ = currentPose.position.z - lastPose.position.z;

            double deltaDistance = sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);

            // Summiere die Distanz über die Zeit
            scalarDistance.data += deltaDistance;

            ROS_INFO_NAMED("Sensibility", "Current Pose x=%f, y=%f, z=%f", currentPose.position.x, currentPose.position.y, currentPose.position.z);
            ROS_INFO_NAMED("Sensibility", "Last Pose x=%f, y=%f, z=%f", lastPose.position.x, lastPose.position.y, lastPose.position.z);
            ROS_INFO_NAMED("Sensibility", "Current distance: %f", scalarDistance.data);

            // Schreibe die Daten in die CSV-Datei
            if (csvFile.is_open())
            {
                csvFile << ros::Time::now() << "," << msgAbsoluteForce.data << "," << scalarDistance.data * 1000 << "\n"; // Distanz in mm
            }

            // Speichere die aktuelle Pose als letzte Pose für die nächste Iteration
            lastPose = currentPose;
        }
    }

    geometry_msgs::Pose Sensibility::getCurrentTCPPose()
    {
        geometry_msgs::Pose currentPose;

        try
        {
            // Hole die Transformation von base_link zu tool0_link
            geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("base_link", "tool0_link", ros::Time(0), ros::Duration(3.0));

            // Konvertiere TransformStamped zu Pose
            currentPose.position.x = transformStamped.transform.translation.x;
            currentPose.position.y = transformStamped.transform.translation.y;
            currentPose.position.z = transformStamped.transform.translation.z;

            currentPose.orientation = transformStamped.transform.rotation;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }

        return currentPose;
    }

    void Sensibility::poseUrCallback(const geometry_msgs::Pose::ConstPtr &poseUR)
    {
        currentPose = *poseUR;
    }

    void Sensibility::measurementPointsCallback(const geometry_msgs::PoseArray::ConstPtr &measurementsPointsMsg)
    {
        poses = *measurementsPointsMsg;
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

        // Get the path to the tars_robot package
        std::string packagePath = ros::package::getPath("tars_robot");
        if (packagePath.empty())
        {
            ROS_ERROR("Could not find package path for tars_robot.");
            return;
        }

        // Save path in the data directory of the tars_robot package
        std::string dataPath = packagePath + "/data";
        ROS_INFO_STREAM("Data will be saved to: " << dataPath);

        // Create the main folder for the measurements
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

            // Create the CSV file
            csvFilePath = poseFolder + "/" + std::to_string(measurementID) + "_Measurement.csv";
            csvFile.open(csvFilePath);
            if (!csvFile.is_open())
            {
                ROS_ERROR_STREAM("Could not create CSV file: " << csvFilePath);
                return;
            }
            csvFile << "Timestamp,AbsoluteForce,Distance(mm)\n"; // CSV header
            ROS_INFO_STREAM("CSV file created: " << csvFilePath);

            // Reset distance and initialize the last pose
            scalarDistance.data = 0;
            lastPose = getCurrentTCPPose(); // Set the initial pose before the measurement

            ROS_INFO("Reset distance to 0 for new measurement point");

            ur10.PTP(pose, ptpVelocity, ptpAcceleration);
            ros::Duration(1.0).sleep(); // Wait to ensure the pose is stable

            startMeasurement = true;

            // LIN movement (measurement)
            ur10.LIN("tool0_link", max_measuring_distance, linearVelocity, linearAcceleration);

            // After the measurement is done, stop the measurement
            startMeasurement = false;

            // Publish force markers after the measurement
            publishForceMarkers(measurementID, msgAbsoluteForce.data, pose);

            csvFile.close();
            ROS_INFO_STREAM("Measurement " << measurementID << " completed and saved to CSV at: " << csvFilePath);

            // Move back
            ros::Duration(2.0).sleep();
            ur10.LIN("tool0_link", -max_measuring_distance, linearVelocity * 10, linearAcceleration * 10, withActiveAirskin);
        }
    }

    void Sensibility::publishForceMarkers(int measurementID, double absoluteForce, const geometry_msgs::Pose& position)
    {
        // Publisher for visualization markers
        ros::Publisher markerPub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

        visualization_msgs::MarkerArray markerArray;

        // Create a marker for the force arrow
        visualization_msgs::Marker arrowMarker;
        arrowMarker.header.frame_id = "base_link";  // Reference frame
        arrowMarker.header.stamp = ros::Time::now();
        arrowMarker.ns = "force_markers";
        arrowMarker.id = measurementID;  // Unique ID for each measurement
        arrowMarker.type = visualization_msgs::Marker::ARROW;
        arrowMarker.action = visualization_msgs::Marker::ADD;

        // Set the arrow pose at the current position
        arrowMarker.pose.position = position.position;

        // Get the orientation of the tool and rotate it to point in the -Z direction
        tf2::Quaternion toolOrientation;
        tf2::fromMsg(position.orientation, toolOrientation);

        // Apply a rotation to the arrow to make it point along the -z axis relative to the tool
        tf2::Quaternion rotation;
        rotation.setRPY(0, M_PI/2, 0);  // Rotate 180 degrees around the x-axis to point -z

        // Combine the tool's orientation with the rotation
        tf2::Quaternion arrowOrientation = toolOrientation * rotation;
        arrowMarker.pose.orientation = tf2::toMsg(arrowOrientation);

        // Set the scale: length is proportional to the force (normalized to 5 N)
        double normalizedForce = std::min(absoluteForce / 20.0, 1.0);  // Normalize to a max of 5 N
        arrowMarker.scale.x = normalizedForce;  // Arrow length based on force
        arrowMarker.scale.y = 0.025;  // Arrow width
        arrowMarker.scale.z = 0.025;  // Arrow height

        // Set the color (e.g., red for the arrow)
        arrowMarker.color.r = 1.0;
        arrowMarker.color.g = 0.0;
        arrowMarker.color.b = 0.0;
        arrowMarker.color.a = 1.0;

        // Add label text for the force value
        visualization_msgs::Marker textMarker;
        textMarker.header.frame_id = "base_link";
        textMarker.header.stamp = ros::Time::now();
        textMarker.ns = "force_labels";
        textMarker.id = measurementID + 1000;  // Unique ID for text (separate from arrow)
        textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        textMarker.action = visualization_msgs::Marker::ADD;

        // Position text slightly above the arrow
        textMarker.pose.position = position.position;
        textMarker.pose.position.z += 0.15;  // Move it up slightly

        textMarker.scale.z = 0.1 / 3;  // Text height
        textMarker.color.r = 0.0;
        textMarker.color.g = 1.0;
        textMarker.color.b = 0.0;
        textMarker.color.a = 1.0;

        // Set the label text to show the force value
        std::stringstream ss;
        ss << "Measurement " << measurementID << ": " << absoluteForce << " N";
        textMarker.text = ss.str();

        // Add both arrow and text markers to the marker array
        markerArray.markers.push_back(arrowMarker);
        markerArray.markers.push_back(textMarker);

        // Publish the marker array
        markerPub.publish(markerArray);

        ROS_INFO("Published force markers for Measurement %d", measurementID);
    }

}

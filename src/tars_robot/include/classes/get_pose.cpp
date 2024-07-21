//get_pose.cpp
#include "get_pose.h"

Pose::Pose(ros::NodeHandle &nh, std::string start_frame, std::string top_frame)
    : nh_(nh), tfListener_(tfBuffer_), start_frame_(start_frame), top_frame_(top_frame) {
    // Initialize the pose variables
    x_ = y_ = z_ = 0.0;
    qx_ = qy_ = qz_ = qw_ = 0.0;
}

Pose::~Pose() {}

void Pose::getPose() {
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer_.lookupTransform(start_frame_, top_frame_, ros::Time(0));

        // Extract translation
        x_ = transformStamped.transform.translation.x;
        y_ = transformStamped.transform.translation.y;
        z_ = transformStamped.transform.translation.z;

        // Extract rotation (quaternion)
        qx_ = transformStamped.transform.rotation.x;
        qy_ = transformStamped.transform.rotation.y;
        qz_ = transformStamped.transform.rotation.z;
        qw_ = transformStamped.transform.rotation.w;

        ROS_INFO("Translation: [%f, %f, %f]", x_, y_, z_);
        ROS_INFO("Rotation (Quaternion): [%f, %f, %f, %f]", qx_, qy_, qz_, qw_);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

void Pose::saveCSV(const std::string &location_to_save) {
    std::ofstream file;
    file.open(location_to_save);

    if (file.is_open()) {
        file << "x,y,z,qx,qy,qz,qw\n";
        file << x_ << "," << y_ << "," << z_ << ",";
        file << qx_ << "," << qy_ << "," << qz_ << "," << qw_ << "\n";
        file.close();
        ROS_INFO("Pose saved to %s", location_to_save.c_str());
    } else {
        ROS_ERROR("Unable to open file: %s", location_to_save.c_str());
    }
}

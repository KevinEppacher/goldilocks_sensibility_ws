//get_pose.h
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>
#include <fstream>

class Pose {
public:
    Pose(ros::NodeHandle &nh, std::string start_frame, std::string top_frame);
    ~Pose();

    void getPose();
    void saveCSV(const std::string &location_to_save);

private:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    std::string start_frame_;
    std::string top_frame_;

    double x_, y_, z_;
    double qx_, qy_, qz_, qw_;
};
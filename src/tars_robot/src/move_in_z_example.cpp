#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

int main(int argc, char** argv)
{
    // ROS-Node initialisieren
    ros::init(argc, argv, "move_along_tool_z_axis");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO_NAMED("move_along_tool_z_axis", "Programm gestartet");

    // MoveIt MoveGroup Interface initialisieren
    static const std::string PLANNING_GROUP = "bdr_ur10"; // Ändern Sie den Planungsgruppennamen entsprechend Ihrem Roboter
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // Aktuelle Pose des Endeffektors abrufen
    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose("tool0_link");

    // Vektor für Wegpunkte erstellen
    std::vector<geometry_msgs::Pose> waypoints;

    // Die Entfernung, die Sie entlang der Z-Achse bewegen möchten
    double distance = 0.1;

    // Neue Zielpose definieren basierend auf der aktuellen Pose
    geometry_msgs::Pose target_pose = current_pose.pose;

    // Erstellen einer Transformation der aktuellen Pose
    tf2::Transform current_transform;
    tf2::fromMsg(current_pose.pose, current_transform);

    // Z-Translation entlang der Werkzeugachse
    tf2::Vector3 translation(0, 0, distance);

    // Translation relativ zur aktuellen Werkzeugorientierung transformieren
    tf2::Vector3 translated_vector = current_transform.getBasis() * translation;

    target_pose.position.x += translated_vector.x();
    target_pose.position.y += translated_vector.y();
    target_pose.position.z += translated_vector.z();

    // Zielpose zu Wegpunkten hinzufügen
    waypoints.push_back(target_pose);

    // Kartesischen Pfad berechnen
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0; // Kein Sprung erlaubt
    const double eef_step = 0.01; // Schrittweite für den Endeffektor
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    // Trajektorie zeitlich parametrisieren
    robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), PLANNING_GROUP);
    rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool success = iptp.computeTimeStamps(rt);
    ROS_INFO("Computed Cartesian path with success: %s (%.2f%% achieved)", success ? "YES" : "NO", fraction * 100.0);

    // Aktualisierte Trajektorie abrufen
    rt.getRobotTrajectoryMsg(trajectory);

    // Trajektorie ausführen
    if (success && fraction > 0.5) // Sicherstellen, dass mindestens 90% der Trajektorie erfolgreich geplant wurden
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        move_group.execute(plan);
    }
    else
    {
        ROS_WARN("Die kartesische Planung war nicht erfolgreich genug.");
    }

    ros::shutdown();
    return 0;
}

#include "Robot.h"

namespace Robot {

    ArticulatedRobot::ArticulatedRobot() : 
        spinner(1), 
        move_group("bdr_ur10"), 
        visual_tools_table("table_link") // For table_link
    {
        // Initialize ROS spinner
        spinner.start();

        // Load MoveIt Visual Tools for table_link
        visual_tools_table.loadRemoteControl();
        visual_tools_table.deleteAllMarkers();

        // Set the text pose for the RViz visual tools
        text_pose = Eigen::Isometry3d::Identity();
        text_pose.translation().z() = 1.25; // Position the text above the robot bases

        // Display initial messages
        visual_tools_table.trigger();

        configureMoveGroup();

    }

    void ArticulatedRobot::configureMoveGroup()
    {
        // Set the maximum planning time
        move_group.setPlanningTime(10.0);

        // Set a different planner algorithm
        move_group.setPlannerId("PRM");

        // Set position and orientation tolerances
        move_group.setGoalPositionTolerance(0.05);

        move_group.setGoalOrientationTolerance(0.1);

        move_group.setNumPlanningAttempts(20);

        move_group.setMaxVelocityScalingFactor(0.01);

        move_group.setMaxAccelerationScalingFactor(0.1);

        move_group.setPoseReferenceFrame("base_link"); // Stellen Sie sicher, dass die Zielpose relativ zu base_link definiert wird

    }

    void ArticulatedRobot::PTP(geometry_msgs::Pose target)
    {
        ROS_INFO("Attempting PTP to target pose: x=%.3f, y=%.3f, z=%.3f, orientation (w,x,y,z)=(%.3f,%.3f,%.3f,%.3f)",
                target.position.x, target.position.y, target.position.z,
                target.orientation.w, target.orientation.x, target.orientation.y, target.orientation.z);

        // Überprüfen der aktuellen Roboterpose
        logRobotState();

        // Visualize the target pose using base_link reference frame
        visual_tools_base.publishAxisLabeled(target, "target_pose_base");
        visual_tools_base.trigger(); // Trigger to display changes in RViz

        // Set the pose target for MoveIt
        move_group.setPoseTarget(target, "tool0_link");

        ROS_INFO("Set pose target for MoveIt.");

        // Plan and move to the target
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        moveit::core::MoveItErrorCode planning_result = move_group.plan(my_plan);
        bool success = (planning_result == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success) {
            ROS_ERROR_STREAM("Planning failed with error code: " << planning_result);
            switch(planning_result.val) {
                case moveit::core::MoveItErrorCode::TIMED_OUT:
                    ROS_ERROR("Planning timed out.");
                    break;
                // Weitere spezifische Fehlerbehandlung basierend auf den verschiedenen Fehlercodes
                default:
                    ROS_ERROR("Unknown planning error occurred.");
                    break;
            }
        } else {
            ROS_INFO("Planning successful. Executing the motion...");

            // Retrieve joint model group for visualization
            const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(move_group.getName());

            // Visualize the plan using table_link reference frame
            visual_tools_table.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
            visual_tools_table.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
            visual_tools_table.trigger(); // Trigger to display trajectory line

            // Execute the plan
            moveit::core::MoveItErrorCode execute_result = move_group.move();

            if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
                ROS_ERROR_STREAM("Execution failed with error code: " << execute_result);
            }
        }
    }



    /// @brief Move the robot end effector in a straight line
    /// @param reference_link The reference link for the linear movement
    /// @param distance The distance to move along the Z-axis
    /// @param linear_velocity The linear velocity for the movement in percentage (0-1)
    void ArticulatedRobot::LIN(const std::string& reference_link, double distance, double linear_velocity)
    {
        // Aktuelle Pose des Endeffektors abrufen
        geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose(reference_link);

        // Set the reference frame to base_link
        move_group.setPoseReferenceFrame("table_link");

        // Transformation der aktuellen Pose
        tf2::Transform current_transform;
        tf2::fromMsg(current_pose.pose, current_transform);

        // Definieren der Translation entlang der Z-Achse
        tf2::Vector3 translation(0, 0, distance);

        // Berechnung der Translation relativ zur aktuellen Werkzeugorientierung
        tf2::Vector3 translated_vector = current_transform.getBasis() * translation;

        // Neue Zielpose basierend auf der Translation definieren
        geometry_msgs::Pose target_pose = current_pose.pose;
        target_pose.position.x += translated_vector.x();
        target_pose.position.y += translated_vector.y();
        target_pose.position.z += translated_vector.z();

        // Visualisierung der Zielpose im Bezug auf den Baseline-Link
        visual_tools_base.publishAxisLabeled(target_pose, "target_pose_base");
        visual_tools_base.trigger(); // Anzeigen der Änderungen in RViz

        // Vektor für Wegpunkte erstellen und die Zielpose hinzufügen
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(target_pose);

        // Kartesischen Pfad berechnen
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0; // Kein Sprung erlaubt
        const double eef_step = 0.01; // Schrittweite für den Endeffektor
        double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        // Trajektorie zeitlich parametrisieren
        robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), move_group.getName());
        rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);

        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        bool success = iptp.computeTimeStamps(rt, linear_velocity);

        // Wenn die Zeitparametrisierung erfolgreich war, aktualisieren Sie die Trajektorie
        if (success)
        {
            rt.getRobotTrajectoryMsg(trajectory);

            // Plan erstellen und ausführen
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;

            // Prüfen, ob der kartesische Pfad erfolgreich war
            if (fraction > 0.3) // Sicherstellen, dass mindestens 90% der Trajektorie erfolgreich geplant wurden
            {
                ROS_INFO_NAMED(className,"Planning successful. Executing the motion...");
                // Trajektorie visualisieren
                const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(move_group.getName());
                visual_tools_table.publishTrajectoryLine(plan.trajectory_, joint_model_group);
                visual_tools_table.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
                visual_tools_table.trigger(); // Anzeigen der Trajektorielinie
                move_group.execute(plan);
            }
            else
            {
                ROS_WARN_NAMED(className,"Planning failed. The Cartesian path was not sufficiently achieved.");
            }
        }
        else
        {
            ROS_WARN_NAMED(className,"Time parameterization failed. Unable to move to target pose.");
        }

        move_group.setPoseReferenceFrame("base_link"); // Set the reference frame back to base_link

    }


    void ArticulatedRobot::planAndVisualize(geometry_msgs::Pose target)
    {
        ROS_INFO("Plan and visualize to target pose: x=%.3f, y=%.3f, z=%.3f, orientation (w,x,y,z)=(%.3f,%.3f,%.3f,%.3f)",
                target.position.x, target.position.y, target.position.z,
                target.orientation.w, target.orientation.x, target.orientation.y, target.orientation.z);

        // Visualize the target pose using base_link reference frame
        visual_tools_base.publishAxisLabeled(target, "target_pose_base");
        visual_tools_base.trigger(); // Trigger to display changes in RViz

        // Set the pose target for MoveIt
        move_group.setPoseTarget(target);

        // Plan and move to the target
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success)
        {
            ROS_INFO("Planning successful. Executing the motion...");

            // Retrieve joint model group for visualization
            const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(move_group.getName());

            // Visualize the trajectory using table_link reference frame
            visual_tools_table.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
            visual_tools_table.trigger(); // Trigger to display trajectory line

            // Execute the plan
            move_group.move();
        }
        else
        {
            ROS_WARN("Planning failed. Unable to move to target pose.");
        }
    }

    void ArticulatedRobot::logRobotState()
    {
        // Get the current robot state
        moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
        
        // Get current joint values
        std::vector<double> joint_values;
        const moveit::core::JointModelGroup* joint_model_group = current_state->getRobotModel()->getJointModelGroup(move_group.getName());
        current_state->copyJointGroupPositions(joint_model_group, joint_values);

        ROS_INFO("Current joint values:");
        for (size_t i = 0; i < joint_values.size(); ++i)
        {
            ROS_INFO("Joint %zu: %.3f", i, joint_values[i]);
        }

        // Check the end effector's current pose
        geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
        ROS_INFO("Current end effector pose: x=%.3f, y=%.3f, z=%.3f, orientation (w,x,y,z)=(%.3f,%.3f,%.3f,%.3f)",
                current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
                current_pose.pose.orientation.w, current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z);
    }
}

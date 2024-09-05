#include "Robot.h"

namespace Robot {

    ArticulatedRobot::ArticulatedRobot(ros::NodeHandle& nodehandler) : 
        spinner(1), 
        move_group("bdr_ur10"), 
        visual_tools_table("table_link"),
        nh(nodehandler)
    {
        airskinStateSub = nh.subscribe("airskin_state", 1, &ArticulatedRobot::airskinStateCallback, this);

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

    void ArticulatedRobot::airskinStateCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        if (!msg->data && moveAllowed)
        {
            move_group.stop();
            moveAllowed = false;
            ROS_WARN("AIRSKIN state changed to false. Stopping the robot.");
        }
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

        move_group.setNumPlanningAttempts(40);

        // move_group.setMaxVelocityScalingFactor(0.01);

        // move_group.setMaxAccelerationScalingFactor(0.1);

        move_group.setPoseReferenceFrame("base_link");
    }

    void ArticulatedRobot::PTP(geometry_msgs::Pose target)
    {
        if (!moveAllowed)
        {
            ROS_WARN("Movement not allowed. Skipping PTP command.");
            moveAllowed = true;
            ROS_INFO("Movement allowed again.");
            return;
        }

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

        moveAllowed = true;
        ROS_INFO("Movement allowed again.");

    }



    /// @brief Move the robot end effector in a straight line
    /// @param reference_link The reference link for the linear movement
    /// @param distance The distance to move along the Z-axis
    /// @param linear_velocity The linear velocity for the movement in percentage (0-1)
    /// @param linear_acceleration The linear acceleration for the movement in percentage (0-1)
    void ArticulatedRobot::LIN(const std::string& reference_link, double distance, double linear_velocity, double linear_acceleration)
    {
        if (!moveAllowed)
        {
            ROS_WARN("Movement not allowed. Skipping PTP command.");
            moveAllowed = true;
            ROS_INFO("Movement allowed again.");
            return;
        }

        // Get the current pose of the end effector
        geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose(reference_link);

        // Set the reference frame to base_link
        move_group.setPoseReferenceFrame("table_link");

        // Transformation of the current pose
        tf2::Transform current_transform;
        tf2::fromMsg(current_pose.pose, current_transform);

        // Define translation along the Z-axis
        tf2::Vector3 translation(0, 0, distance);

        // Calculate translation relative to the current tool orientation
        tf2::Vector3 translated_vector = current_transform.getBasis() * translation;

        // Define a new target pose based on the translation
        geometry_msgs::Pose target_pose = current_pose.pose;
        target_pose.position.x += translated_vector.x();
        target_pose.position.y += translated_vector.y();
        target_pose.position.z += translated_vector.z();

        // Visualize the target pose in relation to the baseline link
        visual_tools_base.publishAxisLabeled(target_pose, "target_pose_base");
        visual_tools_base.trigger(); // Display changes in RViz

        // Create a vector for waypoints and add the initial and target poses
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(current_pose.pose); // Start pose
        waypoints.push_back(target_pose); // End pose

        // Compute the Cartesian path
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0; // No jumps allowed
        const double eef_step = 0.01; // Step size for the end effector
        double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        // If sufficient path is planned
        if (fraction > 0.3) 
        {
            // Parameterize the trajectory timing
            robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), move_group.getName());
            rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);

            trajectory_processing::IterativeParabolicTimeParameterization iptp;
            bool success = iptp.computeTimeStamps(rt, linear_velocity, linear_acceleration);

            if (success)
            {
                rt.getRobotTrajectoryMsg(trajectory);

                // Create and execute the plan
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory_ = trajectory;

                ROS_INFO_NAMED(className, "Planning successful. Executing the motion...");
                const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(move_group.getName());
                visual_tools_table.publishTrajectoryLine(plan.trajectory_, joint_model_group);
                visual_tools_table.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
                visual_tools_table.trigger(); // Show the trajectory line

                move_group.execute(plan);
            }
            else
            {
                ROS_WARN_NAMED(className, "Time parameterization failed. Unable to move to target pose.");
            }
        }
        else
        {
            ROS_WARN_NAMED(className, "Planning failed. The Cartesian path was not sufficiently achieved.");
        }

        ROS_INFO("Movement allowed again.");

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

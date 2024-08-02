#include "Robot.h"

namespace Robot {

    ArticulatedRobot::ArticulatedRobot() : 
        spinner(1), 
        move_group_interface("bdr_ur10"), 
        visual_tools_base("base_link"), // For base_link
        visual_tools_table("table_link") // For table_link
    {
        // Initialize ROS spinner
        spinner.start();

        // Load MoveIt Visual Tools for base_link
        visual_tools_base.loadRemoteControl();
        visual_tools_base.deleteAllMarkers();

        // Load MoveIt Visual Tools for table_link
        visual_tools_table.loadRemoteControl();
        visual_tools_table.deleteAllMarkers();

        // Set the text pose for the RViz visual tools
        text_pose = Eigen::Isometry3d::Identity();
        text_pose.translation().z() = 1.25; // Position the text above the robot base

        // Display initial messages
        visual_tools_base.trigger();
        visual_tools_table.trigger();

        // Set the maximum planning time
        move_group_interface.setPlanningTime(30.0);

        // Set a different planner algorithm
        move_group_interface.setPlannerId("RRTConnectkConfigDefault");

        move_group_interface.setPoseReferenceFrame("base_link");

        // Log the initial robot state
        logRobotState();
    }

    void ArticulatedRobot::PTP(geometry_msgs::Pose target)
    {
        ROS_INFO("Attempting PTP to target pose: x=%.3f, y=%.3f, z=%.3f, orientation (w,x,y,z)=(%.3f,%.3f,%.3f,%.3f)",
                target.position.x, target.position.y, target.position.z,
                target.orientation.w, target.orientation.x, target.orientation.y, target.orientation.z);

        // Visualize the target pose using base_link reference frame
        visual_tools_base.publishAxisLabeled(target, "target_pose_base");
        visual_tools_base.trigger(); // Trigger to display changes in RViz

        // Set the pose target for MoveIt
        move_group_interface.setPoseTarget(target);

        // Plan and move to the target
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success)
        {
            ROS_INFO("Planning successful. Executing the motion...");
            
            // Retrieve joint model group for visualization
            const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(move_group_interface.getName());
            
            // Visualize the plan using table_link reference frame
            visual_tools_table.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
            visual_tools_table.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
            visual_tools_table.trigger(); // Trigger to display trajectory line

            // Execute the plan
            move_group_interface.move();
        }
        else
        {
            ROS_WARN("Planning failed. Unable to move to target pose.");
            debugPlanningFailure(target);
        }
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
        move_group_interface.setPoseTarget(target);

        // Plan and move to the target
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success)
        {
            ROS_INFO("Planning successful. Executing the motion...");

            // Retrieve joint model group for visualization
            const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(move_group_interface.getName());

            // Visualize the trajectory using table_link reference frame
            visual_tools_table.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
            visual_tools_table.trigger(); // Trigger to display trajectory line

            // Execute the plan
            move_group_interface.move();
        }
        else
        {
            ROS_WARN("Planning failed. Unable to move to target pose.");
            debugPlanningFailure(target);
        }
    }

    void ArticulatedRobot::logRobotState()
    {
        // Get the current robot state
        moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
        
        // Get current joint values
        std::vector<double> joint_values;
        const moveit::core::JointModelGroup* joint_model_group = current_state->getRobotModel()->getJointModelGroup(move_group_interface.getName());
        current_state->copyJointGroupPositions(joint_model_group, joint_values);

        ROS_INFO("Current joint values:");
        for (size_t i = 0; i < joint_values.size(); ++i)
        {
            ROS_INFO("Joint %zu: %.3f", i, joint_values[i]);
        }

        // Check the end effector's current pose
        geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();
        ROS_INFO("Current end effector pose: x=%.3f, y=%.3f, z=%.3f, orientation (w,x,y,z)=(%.3f,%.3f,%.3f,%.3f)",
                current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
                current_pose.pose.orientation.w, current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z);
    }

    void ArticulatedRobot::debugPlanningFailure(const geometry_msgs::Pose& target_pose)
    {
        // Log detailed information about the planning scene and robot state
        logRobotState();

        // Check the constraints and other factors
        moveit_msgs::Constraints constraints = move_group_interface.getPathConstraints();
        ROS_INFO("Path constraints are set: %s", constraints.name.c_str());

        // Log planning scene details if available
        moveit::planning_interface::PlanningSceneInterface psi;
        std::map<std::string, moveit_msgs::CollisionObject> co = psi.getObjects();
        ROS_INFO("Number of collision objects in the planning scene: %zu", co.size());

        for (const auto& obj : co)
        {
            ROS_INFO("Collision Object: %s", obj.first.c_str());
        }

        // Get the robot's current planning frame
        std::string planning_frame = move_group_interface.getPlanningFrame();
        ROS_INFO("Planning frame: %s", planning_frame.c_str());

        // Log the name of the end effector link
        std::string end_effector_link = move_group_interface.getEndEffectorLink();
        ROS_INFO("End effector link: %s", end_effector_link.c_str());

        // Check the current allowed collision matrix
        planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
        psm->requestPlanningSceneState(); // Ensure we have the latest planning scene
        collision_detection::AllowedCollisionMatrix acm = psm->getPlanningScene()->getAllowedCollisionMatrix();

        std::vector<std::string> entry_names;
        acm.getAllEntryNames(entry_names);

        ROS_INFO("Allowed collision matrix size: %zu", entry_names.size());

        for (const std::string& link_name : entry_names)
        {
            for (const std::string& other_link_name : entry_names)
            {
                collision_detection::AllowedCollision::Type type;
                if (acm.getEntry(link_name, other_link_name, type))
                {
                    ROS_INFO("Link: %s with %s has allowed collision type: %d", link_name.c_str(), other_link_name.c_str(), type);
                }
            }
        }

        // Additional debugging: check if the target is reachable by using an IK solver
        moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
        const moveit::core::JointModelGroup* joint_model_group = current_state->getRobotModel()->getJointModelGroup(move_group_interface.getName());

        bool ik_success = current_state->setFromIK(joint_model_group, target_pose, 0.1);  // Updated the deprecated method
        if (!ik_success)
        {
            ROS_WARN("Inverse Kinematics could not find a solution for the target pose.");
        }
        else
        {
            ROS_INFO("Inverse Kinematics found a valid solution for the target pose.");
        }
    }

}

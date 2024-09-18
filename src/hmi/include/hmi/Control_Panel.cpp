#include "Control_Panel.h"

ControlPanel::ControlPanel(ros::NodeHandle &N) : nh(N)
{
    indexControlURProgramPub = nh.advertise<std_msgs::Int32>("hmi/control_ur_program", 10);
    poseSub = nh.subscribe("measurement_points", 1, &ControlPanel::poseCallback, this);
}

void ControlPanel::render()
{
    // Set button size
    ImVec2 button_size(300, 100); // Width, Height

    // Set position and create Setup button
    ImGui::SetCursorPos(ImVec2(50, 50)); // X, Y
    if (ImGui::Button("Setup", button_size))
    {
        setupButtonCallback();
    }


    // Set position and create Begin Measurement button
    ImGui::SetCursorPos(ImVec2(50, 200)); // X, Y
    if (ImGui::Button("Begin External Control", button_size))
    {
        for (int i = 0; i < target_poses.poses.size(); ++i)
        {
            geometry_msgs::Pose pose = handlePose(i, target_poses);
            moveRobotToPose("ur10_manipulator", pose);
        }
    }

    // Set position and create default button
    ImGui::SetCursorPos(ImVec2(50, 350)); // X, Y
    if (ImGui::Button("Default QUANTEC", button_size))
    {
        defaultQUANTECCallback();
    }

    // Set position and create default button
    ImGui::SetCursorPos(ImVec2(50, 500)); // X, Y
    if (ImGui::Button("Default UR10", button_size))
    {
        defaultUR10Callback();
    }
}

void ControlPanel::setupButtonCallback()
{
    std_msgs::Int32 msg;
    msg.data = setupValue;
    indexControlURProgramPub.publish(msg);
    ROS_INFO("Setup button pressed, published value: %d", setupValue);
}

void ControlPanel::beginExternalControlButtonCallback()
{
    std_msgs::Int32 msg;
    msg.data = externalControlValue;
    indexControlURProgramPub.publish(msg);
    ROS_INFO("Begin Measurement button pressed, published value: %d", externalControlValue);
}

void ControlPanel::moveRobotToHome(const std::string& planning_group, const std::string& target_pose)
{
    // Initialize Move Group Interface for the robot
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);

    // Set goal tolerance for the robot to prevent aborts due to small errors
    double goal_tolerance;

    nh.param("goal_tolerance", goal_tolerance, 0.1);  // Default value is 0.1 if parameter not set
    move_group.setGoalTolerance(goal_tolerance);

    // Set the target pose to the home position for the specified robot
    move_group.setNamedTarget(target_pose);

    // Start movement asynchronously
    moveit::planning_interface::MoveItErrorCode result = move_group.asyncMove();

    // Check if the command was successfully sent
    if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("%s robot is moving to home position.", planning_group.c_str());
    }
    else
    {
        ROS_ERROR("Failed to start movement for %s robot. Error code: %d", planning_group.c_str(), result.val);
    }
}


void ControlPanel::defaultUR10Callback()
{
    std_msgs::Int32 msg;
    msg.data = defaultValue;
    indexControlURProgramPub.publish(msg);
    ROS_INFO("Default Setting button pressed, published value: %d", defaultValue);

    // Then move UR10
    ROS_WARN("Moving UR10 to home...");
    moveRobotToHome("ur10_manipulator", "ur10_home");
}

void ControlPanel::defaultQUANTECCallback()
{
    std_msgs::Int32 msg;
    msg.data = defaultValue;
    indexControlURProgramPub.publish(msg);
    ROS_INFO("Default Setting button pressed, published value: %d", defaultValue);

    // Move Quantec first
    ROS_WARN("Moving Quantec to home...");
    moveRobotToHome("quantec_manipulator", "quantec_home");
}

void ControlPanel::poseCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    if(!msg->poses.empty()){
        target_poses = *msg;
    }
}

void ControlPanel::moveRobotToPose(const std::string& planning_group, const geometry_msgs::Pose& target_pose)
{
    std_msgs::Int32 msg;
    msg.data = externalControlValue;
    indexControlURProgramPub.publish(msg);
    ROS_WARN("Begin Measurement button pressed, published value: %d", externalControlValue);

    moveit::planning_interface::MoveGroupInterface move_group(planning_group);

    double goal_position_tolerance = 0.01; 
    nh.param("goal_position_tolerance", goal_position_tolerance, goal_position_tolerance);
    move_group.setGoalPositionTolerance(goal_position_tolerance);
    ROS_INFO("Goal position tolerance set to: %f meters", goal_position_tolerance);

    double goal_orientation_tolerance = 0.01; 
    nh.param("goal_orientation_tolerance", goal_orientation_tolerance, goal_orientation_tolerance);
    move_group.setGoalOrientationTolerance(goal_orientation_tolerance);
    ROS_INFO("Goal orientation tolerance set to: %f radians", goal_orientation_tolerance);

    double planning_time = 5.0;
    nh.param("planning_time", planning_time, planning_time);
    move_group.setPlanningTime(planning_time);
    ROS_INFO("Planning time set to: %f seconds", planning_time);

    int planning_attempts = 5;
    nh.param("planning_attempts", planning_attempts, planning_attempts);
    move_group.setNumPlanningAttempts(planning_attempts);
    ROS_INFO("Planning attempts set to: %d", planning_attempts);

    move_group.setPoseReferenceFrame("base_link");

    move_group.setPoseTarget(target_pose);

    ROS_WARN("Moving robot to target pose... %f %f %f", target_pose.position.x, target_pose.position.y, target_pose.position.z);

    move_group.asyncMove();

    move_group.clearPoseTargets();
}

geometry_msgs::Pose ControlPanel::handlePose(int index, const geometry_msgs::PoseArray& poses)
{
    // Simply return the pose at the given index
    return poses.poses[index];
}






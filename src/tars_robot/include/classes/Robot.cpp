#include "Robot.h"

namespace Robot
{

    ArticulatedRobot::ArticulatedRobot(ros::NodeHandle &nodeHandler) : spinner(1),
                                                                       moveGroup("bdr_ur10"),
                                                                       visualToolsTable("table_link"),
                                                                       nh(nodeHandler)
    {
        loadParameters(); // Load parameters from YAML file

        airskinStateSub = nh.subscribe("airskin_state", 1, &ArticulatedRobot::airskinStateCallback, this);

        // Initialize ROS spinner
        spinner.start();

        // Load MoveIt Visual Tools for table_link
        visualToolsTable.loadRemoteControl();
        visualToolsTable.deleteAllMarkers();

        // Set the text pose for the RViz visual tools
        textPose = Eigen::Isometry3d::Identity();
        textPose.translation().z() = 1.25; // Position the text above the robot bases

        // Display initial messages
        visualToolsTable.trigger();

        configureMoveGroup(); // Configure MoveGroupInterface settings
    }

    void ArticulatedRobot::loadParameters()
    {
        // Load parameters from the parameter server (YAML file)
        nh.param("robot_motion/acceptable_fraction", acceptableFraction, 0.9);
        nh.param("robot_motion/planner_id", plannerId, std::string("PRM"));
        nh.param("robot_motion/planning_attempts", planningAttempts, 100);
        nh.param("robot_motion/planning_time", planningTime, 20.0);
        nh.param("robot_motion/goal_position_tolerance", goalPositionTolerance, 0.01);
        nh.param("robot_motion/goal_orientation_tolerance", goalOrientationTolerance, 0.05);
    }

    void ArticulatedRobot::airskinStateCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        if (!msg->data && moveAllowed)
        {
            moveGroup.stop();
            // moveAllowed = false;
            ROS_WARN("AIRSKIN state changed to false. Stopping the robot.");
        }
    }

    void ArticulatedRobot::configureMoveGroup()
    {
        // Set the maximum planning time
        moveGroup.setPlanningTime(planningTime);

        // Set the planner algorithm
        moveGroup.setPlannerId(plannerId);

        // Set position and orientation tolerances
        moveGroup.setGoalPositionTolerance(goalPositionTolerance);
        moveGroup.setGoalOrientationTolerance(goalOrientationTolerance);

        // Set the number of planning attempts
        moveGroup.setNumPlanningAttempts(planningAttempts);

        moveGroup.setPoseReferenceFrame("base_link");
    }

    void ArticulatedRobot::PTP(geometry_msgs::Pose target, double velocityScaling, double accelerationScaling)
    {
        // if (!moveAllowed)
        // {
        //     ROS_WARN("Movement not allowed. Skipping PTP command.");
        //     moveAllowed = true;
        //     ROS_INFO("Movement allowed again.");
        // }

        // disableOctomapCollision();

        // Apply velocity and acceleration scaling
        moveGroup.setMaxVelocityScalingFactor(velocityScaling);
        moveGroup.setMaxAccelerationScalingFactor(accelerationScaling);
        ROS_INFO_NAMED(className, "PTP Velocity: %.3f, PTP Acceleration: %.3f", velocityScaling, accelerationScaling);
        ROS_INFO("Attempting PTP to target pose: x=%.3f, y=%.3f, z=%.3f, orientation (w,x,y,z)=(%.3f,%.3f,%.3f,%.3f)",
                target.position.x, target.position.y, target.position.z,
                target.orientation.w, target.orientation.x, target.orientation.y, target.orientation.z);

        // Check current robot state
        // logRobotState();

        // Visualize the target pose using base_link reference frame
        visualToolsBase.publishAxisLabeled(target, "target_pose_base");
        visualToolsBase.trigger(); // Trigger to display changes in RViz

        // Set the pose target for MoveIt
        moveGroup.setPoseTarget(target, "tool0_link");

        ROS_INFO("Set pose target for MoveIt.");

        // Plan and move to the target
        moveit::planning_interface::MoveGroupInterface::Plan myPlan;
        moveit::core::MoveItErrorCode planningResult = moveGroup.plan(myPlan);
        bool success = (planningResult == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success)
        {
            ROS_ERROR_STREAM("Planning failed with error code: " << planningResult);
            switch (planningResult.val)
            {
            case moveit::core::MoveItErrorCode::TIMED_OUT:
                ROS_ERROR("Planning timed out.");
                break;
            default:
                ROS_ERROR("Unknown planning error occurred.");
                break;
            }
        }
        else
        {
            ROS_INFO("Planning successful. Executing the motion...");

            // Retrieve joint model group for visualization
            const moveit::core::JointModelGroup *jointModelGroup = moveGroup.getCurrentState()->getJointModelGroup(moveGroup.getName());

            // Visualize the plan using table_link reference frame
            visualToolsTable.publishTrajectoryLine(myPlan.trajectory_, jointModelGroup);
            visualToolsTable.publishText(textPose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
            visualToolsTable.trigger(); // Trigger to display trajectory line

            // Execute the plan
            moveit::core::MoveItErrorCode executeResult = moveGroup.move();

            if (executeResult != moveit::core::MoveItErrorCode::SUCCESS)
            {
                ROS_ERROR_STREAM("Execution failed with error code: " << executeResult);
            }
        }

        // enableOctomapCollision();

        // moveAllowed = true;
    }

    void ArticulatedRobot::LIN(const std::string &referenceLink, double distance, double velocityScaling, double accelerationScaling, bool withActiveAirskin) 
    {
        if (withActiveAirskin)
        {
            moveAllowed = false;
        }

        // disableOctomapCollision();

        // Apply velocity and acceleration scaling
        moveGroup.setMaxVelocityScalingFactor(velocityScaling);
        moveGroup.setMaxAccelerationScalingFactor(accelerationScaling);
        ROS_INFO_NAMED(className, "LIN Velocity: %f, LIN Acceleration: %f", velocityScaling, accelerationScaling);
        ROS_INFO_NAMED(className, "Moving %f meters along the Z-axis in the %s reference frame.", distance, referenceLink.c_str());

        // Get the current pose of the end effector
        geometry_msgs::PoseStamped currentPose = moveGroup.getCurrentPose(referenceLink);

        // Set the reference frame to base_link
        moveGroup.setPoseReferenceFrame("table_link");

        // Transformation of the current pose
        tf2::Transform currentTransform;
        tf2::fromMsg(currentPose.pose, currentTransform);

        // Define translation along the Z-axis
        tf2::Vector3 translation(0, 0, distance);
        tf2::Vector3 translatedVector = currentTransform.getBasis() * translation;

        // Define a new target pose based on the translation
        geometry_msgs::Pose targetPose = currentPose.pose;
        targetPose.position.x += translatedVector.x();
        targetPose.position.y += translatedVector.y();
        targetPose.position.z += translatedVector.z();

        // Visualize the target pose in relation to the baseline link
        visualToolsBase.publishAxisLabeled(targetPose, "target_pose_base");
        visualToolsBase.trigger(); // Display changes in RViz

        // Create a vector for waypoints and add only the target pose (no initial pose)
        std::vector<geometry_msgs::Pose> waypoints;
        // waypoints.push_back(currentPose.pose);
        waypoints.push_back(targetPose); // Only the end pose

        // Compute the Cartesian path
        moveit_msgs::RobotTrajectory trajectory;
        const double jumpThreshold = 0.0; // No jumps allowed
        const double eefStep = 0.01;      // Step size for the end effector
        double fraction = moveGroup.computeCartesianPath(waypoints, eefStep, jumpThreshold, trajectory);

        // If sufficient path is planned
        if (fraction > acceptableFraction)
        {
            // Parameterize the trajectory timing
            robot_trajectory::RobotTrajectory rt(moveGroup.getCurrentState()->getRobotModel(), moveGroup.getName());
            rt.setRobotTrajectoryMsg(*moveGroup.getCurrentState(), trajectory);

            trajectory_processing::IterativeParabolicTimeParameterization iptp;
            bool success = iptp.computeTimeStamps(rt, velocityScaling, accelerationScaling);

            if (success)
            {
                rt.getRobotTrajectoryMsg(trajectory);

                // Create and execute the plan
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory_ = trajectory;

                ROS_INFO_NAMED(className, "Planning successful. Executing the motion...");
                const moveit::core::JointModelGroup *jointModelGroup = moveGroup.getCurrentState()->getJointModelGroup(moveGroup.getName());
                visualToolsTable.publishTrajectoryLine(plan.trajectory_, jointModelGroup);
                visualToolsTable.publishText(textPose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
                visualToolsTable.trigger(); // Show the trajectory line

                moveGroup.execute(plan);
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

        moveAllowed = true;

        // enableOctomapCollision();

        moveGroup.setPoseReferenceFrame("base_link");
    }

    void ArticulatedRobot::disableOctomapCollision()
    {
        moveit::planning_interface::PlanningSceneInterface planningSceneInterface;

        // Holen Sie sich alle bekannten Objekt-IDs in der Planungsumgebung
        std::vector<std::string> object_ids = planningSceneInterface.getKnownObjectNames();

        // Wenn es bekannte Objekte gibt, diese aus der Planungsumgebung entfernen
        if (!object_ids.empty())
        {
            planningSceneInterface.removeCollisionObjects(object_ids);
            ROS_INFO("Alle Kollisionsobjekte (einschließlich Octomap, falls vorhanden) wurden entfernt.");
        }
        else
        {
            ROS_INFO("Keine Objekte zum Entfernen gefunden.");
        }
    }


    void ArticulatedRobot::enableOctomapCollision()
    {
        moveit::planning_interface::PlanningSceneInterface planningSceneInterface;

        // Aktualisiere die Kollisionserkennung: Füge die Octomap wieder hinzu
        moveit_msgs::CollisionObject octomap;
        octomap.id = "";  // Setze den richtigen ID-Namen der Octomap
        octomap.operation = moveit_msgs::CollisionObject::ADD;

        planningSceneInterface.applyCollisionObject(octomap);
        ROS_INFO("Octomap-Kollisionserkennung aktiviert.");
    }


    void ArticulatedRobot::logRobotState()
    {
        // Get the current robot state
        moveit::core::RobotStatePtr currentState = moveGroup.getCurrentState();

        // Get current joint values
        std::vector<double> jointValues;
        const moveit::core::JointModelGroup *jointModelGroup = currentState->getRobotModel()->getJointModelGroup(moveGroup.getName());
        currentState->copyJointGroupPositions(jointModelGroup, jointValues);

        ROS_INFO("Current joint values:");
        for (size_t i = 0; i < jointValues.size(); ++i)
        {
            ROS_INFO("Joint %zu: %.3f", i, jointValues[i]);
        }

        // Check the end effector's current pose
        geometry_msgs::PoseStamped currentPose = moveGroup.getCurrentPose();
        ROS_INFO("Current end effector pose: x=%.3f, y=%.3f, z=%.3f, orientation (w,x,y,z)=(%.3f,%.3f,%.3f,%.3f)",
                 currentPose.pose.position.x, currentPose.pose.position.y, currentPose.pose.position.z,
                 currentPose.pose.orientation.w, currentPose.pose.orientation.x, currentPose.pose.orientation.y, currentPose.pose.orientation.z);
    }
}

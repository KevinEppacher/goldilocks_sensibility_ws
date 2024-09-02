#include <ros/ros.h>
#include <std_srvs/Trigger.h> // Hypothetical correct service header
#include <std_msgs/Bool.h>    // Header for publishing a Bool message
#include <ur_msgs/IOStates.h> // Header for subscribing to /ur_hardware_interface/io_states

class ForceTorqueSensor
{
public:
    ForceTorqueSensor()
    {
        // Create ROS NodeHandle
        ros::NodeHandle nh;

        // Subscribe to the IO states topic
        ioStatesSub = nh.subscribe("/ur_hardware_interface/io_states", 10, &ForceTorqueSensor::ioStatesCallback, this);

        // Create a service client for the correct service type
        stopClient = nh.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/stop");

        // Wait for the service to become available
        ROS_INFO("Waiting for /ur_hardware_interface/dashboard/stop service to become available...");
        stopClient.waitForExistence();
        ROS_INFO("/ur_hardware_interface/dashboard/stop service is now available.");

        // Create a publisher for airskin_state
        airskinPub = nh.advertise<std_msgs::Bool>("airskin_state", 10);

        // Initialize the flag for state tracking
        previousAirskinState = true; // Assuming the initial state is true
    }

private:
    ros::Subscriber ioStatesSub;
    ros::ServiceClient stopClient;
    ros::Publisher airskinPub;

    bool previousAirskinState; // To track the previous state of airskin

    void ioStatesCallback(const ur_msgs::IOStates::ConstPtr& msg)
    {
        // Directly access the states of pin 0 and pin 1
        bool pin0State = msg->digital_in_states[0].state;
        bool pin1State = msg->digital_in_states[1].state;

        // Determine the current airskin state
        bool currentAirskinState = pin0State && pin1State;

        // Only check for state change from true to false
        if (previousAirskinState && !currentAirskinState)
        {
            // Publish the airskin state as false
            std_msgs::Bool airskinStateMsg;
            airskinStateMsg.data = false;
            airskinPub.publish(airskinStateMsg);
            ROS_INFO("airskin_state changed to false.");

            // Stop the robot's current motion with the correct request type
            std_srvs::Trigger srv;
            if (stopClient.call(srv))
            {
                if (srv.response.success)
                {
                    ROS_INFO("Robot motion stopped because airskin_state changed to false.");
                }
                else
                {
                    ROS_ERROR("Service call succeeded, but failed to stop robot: %s", srv.response.message.c_str());
                }
            }
            else
            {
                ROS_ERROR("Failed to call /ur_hardware_interface/dashboard/stop service.");
            }
        }

        // Update the previous state
        previousAirskinState = currentAirskinState;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ft_sensor_node");
    ForceTorqueSensor forceTorqueSensor;

    ros::spin();

    return 0;
}

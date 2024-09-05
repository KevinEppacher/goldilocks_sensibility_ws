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

        // Create a publisher for airskin_state
        airskinPub = nh.advertise<std_msgs::Bool>("airskin_state", 10);

    }

private:
    ros::Subscriber ioStatesSub;
    ros::Publisher airskinPub;

    void ioStatesCallback(const ur_msgs::IOStates::ConstPtr& msg)
    {
        // Directly access the states of pin 0 and pin 1
        bool pin0State = msg->digital_in_states[1].state;
        bool pin1State = msg->digital_in_states[2].state;

        // Determine the current airskin state
        bool currentAirskinState = pin0State && pin1State;
        ROS_INFO("Current airskin state: %d", currentAirskinState);

        // Publish the airskin state as false
        std_msgs::Bool airskinStateMsg;

        // Only check for state change from true to false
        if (!currentAirskinState)
        {
            airskinStateMsg.data = false;
            airskinPub.publish(airskinStateMsg);
        }
        else
        {
            airskinStateMsg.data = true;
            airskinPub.publish(airskinStateMsg);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ft_sensor_node");
    ForceTorqueSensor forceTorqueSensor;

    ros::spin();

    return 0;
}

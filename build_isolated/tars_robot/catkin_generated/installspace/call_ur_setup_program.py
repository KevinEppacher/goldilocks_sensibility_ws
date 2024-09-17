import rospy
from ur_dashboard_msgs.srv import Load, LoadRequest
from std_srvs.srv import Trigger
from std_msgs.msg import Int32

# Global variable to store the index received from the subscriber
program_index = -1  # Start with an invalid value to ensure the first update triggers the action
last_program_index = -1

def control_callback(msg):
    global program_index
    program_index = msg.data

def load_program(action):
    rospy.wait_for_service('/ur_hardware_interface/dashboard/load_program')
    rospy.wait_for_service('/ur_hardware_interface/dashboard/play')
    rospy.wait_for_service('/ur_hardware_interface/dashboard/stop')

    try:
        if action == "load_and_start":
            rospy.sleep(1)
            # Load program
            load_program_service = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
            request = LoadRequest()
            request.filename = '/programs/Sensitivity_Measurement/setup_robot.urp'
            response = load_program_service(request)
            if response.success:
                rospy.loginfo("Setup program loaded successfully.")
                rospy.sleep(1)
                # Play program
                play_service = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
                play_response = play_service()
                if play_response.success:
                    rospy.loginfo("Setup program started successfully.")
                else:
                    rospy.logwarn("Failed to start setup program.")
            else:
                rospy.logwarn("Failed to load program.")
        elif action == "stop":
            rospy.sleep(1)
            # Stop program
            stop_service = rospy.ServiceProxy('/ur_hardware_interface/dashboard/stop', Trigger)
            stop_response = stop_service()
            if stop_response.success:
                rospy.loginfo("Setup program stopped successfully.")
            else:
                rospy.logwarn("Failed to stop setup program.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def main():
    global program_index, last_program_index
    rospy.init_node('load_program_node')
    rospy.Subscriber('/hmi/control_ur_program', Int32, control_callback)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        if program_index != last_program_index:
            if program_index == 0:
                load_program("stop")
            elif program_index == 1:
                load_program("load_and_start")
            elif program_index == 2:
                load_program("stop")
            else:
                rospy.logwarn("Unknown program index received: %d", program_index)
            last_program_index = program_index
        rate.sleep()

if __name__ == "__main__":
    main()

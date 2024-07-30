import rospy
import sys
from ur_dashboard_msgs.srv import Load, LoadRequest
from std_srvs.srv import Trigger

def load_program(action):
    rospy.init_node('load_program_node')
    
    rospy.wait_for_service('/ur_hardware_interface/dashboard/load_program')
    rospy.wait_for_service('/ur_hardware_interface/dashboard/play')
    rospy.wait_for_service('/ur_hardware_interface/dashboard/stop')
    
    try:
        # Load program
        rospy.sleep(1)
        load_program_service = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
        request = LoadRequest()
        request.filename = '/programs/Sensitivity_Measurement/measure_sensibility.urp'
        response = load_program_service(request)
        if response.success:
            rospy.loginfo("Measurement program loaded successfully.")
        else:
            rospy.logwarn("Failed to load measurement program.")
            return
        
        if action == "start":
            # Play program
            rospy.sleep(5)
            play_service = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
            play_response = play_service()
            if play_response.success:
                rospy.loginfo("Measurement program started successfully.")
            else:
                rospy.logwarn("Failed to start measurement program.")
        
        elif action == "stop":
            # Stop program
            rospy.sleep(3)
            stop_service = rospy.ServiceProxy('/ur_hardware_interface/dashboard/stop', Trigger)
            stop_response = stop_service()
            if stop_response.success:
                rospy.loginfo("Measurement program stopped successfully.")
            else:
                rospy.logwarn("Failed to stop measurement program.")
        else:
            rospy.logerr("Invalid action specified. Use 'start' or 'stop'.")
    
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        rospy.logerr("Usage: rosrun your_package call_ur_setup_program.py <start|stop>")
    else:
        action = sys.argv[1]
        load_program(action)

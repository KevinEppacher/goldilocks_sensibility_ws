import rospy
from std_srvs.srv import Trigger

def deactivate_freedrive_mode():
    try:
        # Warte auf den Dienst
        rospy.wait_for_service('/ur_hardware_interface/dashboard/stop')
        
        # Erzeuge einen Dienstaufruf, um den Freedrive-Modus zu deaktivieren
        stop_service = rospy.ServiceProxy('/ur_hardware_interface/dashboard/stop', Trigger)
        
        # Rufe den Dienst auf
        stop_response = stop_service()
        if stop_response.success:
            rospy.loginfo("Freedrive mode deactivated.")
        else:
            rospy.logerr("Failed to deactivate Freedrive mode: %s" % stop_response.message)
            return False
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False
    
    return True

def activate_external_control():
    try:
        # Warte auf den Dienst
        rospy.wait_for_service('/ur_hardware_interface/resend_robot_program')
        
        # Erzeuge einen Dienstaufruf, um den External Control Modus zu aktivieren
        resend_program_service = rospy.ServiceProxy('/ur_hardware_interface/resend_robot_program', Trigger)
        
        # Rufe den Dienst auf
        resend_program_response = resend_program_service()
        if resend_program_response.success:
            rospy.loginfo("External Control mode activated.")
        else:
            rospy.logerr("Failed to activate External Control mode: %s" % resend_program_response.message)
            return False
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False
    
    return True

if __name__ == '__main__':
    rospy.init_node('freedrive_mode_control_node')
    
    if deactivate_freedrive_mode():
        rospy.loginfo("Proceeding to activate External Control mode...")
        activate_external_control()

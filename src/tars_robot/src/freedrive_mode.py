#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from ur_dashboard_msgs.msg import RobotMode
from ur_dashboard_msgs.srv import IsProgramRunning

def robot_mode_callback(msg):
    if msg.mode == 6:
        rospy.loginfo("The robot is in Freedrive Mode.")
    else:
        rospy.loginfo("The robot is in mode: %d" % msg.mode)

def activate_freedrive_mode():
    rospy.init_node('activate_freedrive_mode_node', anonymous=True)
    pub = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=10)
    rospy.Subscriber("/ur_hardware_interface/robot_mode", RobotMode, robot_mode_callback)
    rospy.sleep(1)  # Warte eine Sekunde, um sicherzustellen, dass der Publisher bereit ist
    script = String()
    script.data = 'def myProg():\n  freedrive_mode()\nend\n'
    pub.publish(script)
    rospy.loginfo("Freedrive mode activated.")
    rospy.spin()

if __name__ == '__main__':
    try:
        activate_freedrive_mode()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class Sensibility:
    def __init__(self):
        # Publisher für das Senden von URScript-Befehlen
        self.ur_script_publisher = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=10)

    def zero_ft_sensor(self):
        # Erstelle eine Nachricht für den URScript-Befehl
        script_command = String()
        script_command.data = "rq_zero_sensor()"
        rospy.loginfo("Zeroing the force/torque sensor...")

        # Warte, bis die Verbindung zum Publisher hergestellt ist
        rate = rospy.Rate(10)
        while self.ur_script_publisher.get_num_connections() == 0:
            rate.sleep()
            rospy.loginfo("Waiting for the URScript publisher to connect...")

        # Veröffentliche den URScript-Befehl
        self.ur_script_publisher.publish(script_command)
        rospy.loginfo("URScript command sent: %s", script_command.data)

if __name__ == "__main__":
    # Initialisiere die ROS-Node
    rospy.init_node('sensibility_node', anonymous=True)

    # Erstelle eine Instanz der Sensibility-Klasse
    sensibility = Sensibility()

    # Führe den Null-Befehl aus
    sensibility.zero_ft_sensor()

    # ROS-Node aktiv halten
    rospy.spin()

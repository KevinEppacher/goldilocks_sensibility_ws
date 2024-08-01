import subprocess
import rospy

def launch_ros_file():
    rospy.init_node('launch_demo_node', anonymous=True)
    try:
        # Der Befehl, um das Launch-File zu starten
        rospy.sleep(2)
        command = 'roslaunch bdr_ur10_moveit demo.launch'
        
        # ROSLAUNCH-Befehl in einer Shell ausführen
        subprocess.call(command, shell=True)
        
    except Exception as e:
        rospy.logerr("Failed to launch demo: %s", str(e))

if __name__ == "__main__":
    launch_ros_file()

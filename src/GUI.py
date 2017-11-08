import rospy
import csv
from std_msgs.msg import Int32
from std_msgs.msg import String
from gestureClasses import *
from gestureInfo import *

def getData():
    rospy.init_node('crazyFrog/GUI_data_reciever', anonymous=True)
    sub1 = rospy.Subscriber('crazyFrog/current_gesture', Int32 , DO-STUFF1)
    sub2 = rospy.Subscriber('crazyFrog/current_program', MacroRequest, DO-STUFF2)
    sub3 = rospy.Subscriber('crazyFrog/compiler_data', CompilerData, DO-STUFF3)
    rospy.spin()
	
if __name__ == '__main__':
    try:
        getData()
    except rospy.ROSInterruptException:
        pass

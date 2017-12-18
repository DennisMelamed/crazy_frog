#!/usr/bin/env python


## tester for the gesture Compiler that publishes std_msgs/int32 which are commands subscribed to by the gestureCompiler
## to the 'current_gesture' topic

import rospy
from gestureInfo import *
from std_msgs.msg import Int32

#command_list = [12,14,0,11,12,14,1,11,19,14,1,14,0,11,11,16,14,3,11,13,14,1,11,11,11]#,10,14,0,11]
command_list=[NOP,SetNumberVar,RecordMacro,Digit,0,END,Digit,1,END,RecordMacro,Digit,0,END,RecordMacro,CallNumberVar,RecordMacro,Digit,0,END,MoveZ,Digit,1,Digit,0,END,END,Repeat,Digit,3,END,CallMacro,Digit,1,END,END,END]
#					22			12		 14    0   11  14  1  11
#command_list = [NOP,SetNumberVar,RecordMacro,Digit,0,END,Digit,1,END,RecordMacro,CallNumberVar,RecordMacro,Digit,0,END]# SetNumberVar,RecordMacro,Digit,0,END,Digit,1,END]
# for command in command_list:
#    	processGesture(command)

def talker():
    pub = rospy.Publisher('crazyFrog/current_gesture', Int32, queue_size=10)
    rospy.init_node('compiler_tester', anonymous=True)
    rate = rospy.Rate(9) # 1hz
    for command in command_list:
        rospy.loginfo(command)
        pub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

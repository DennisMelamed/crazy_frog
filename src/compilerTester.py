#!/usr/bin/env python


## tester for the gesture Compiler that publishes std_msgs/int32 which are commands subscribed to by the gestureCompiler
## to the 'current_gesture' topic

import rospy
from std_msgs.msg import int32

command_list = [12,14,0,11,12,14,1,11,19,14,1,14,0,11,11,16,14,3,11,13,14,1,11,11,11]#,10,14,0,11]
for command in command_list:
   	processGesture(command)

def talker():
    pub = rospy.Publisher('crazyFrog/current_esture', int32, queue_size=10)
    rospy.init_node('crazyFrog/compiler_tester', anonymous=True)
    rate = rospy.Rate(1) # 2hz
    for command in command_list:
        rospy.loginfo(command)
        pub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

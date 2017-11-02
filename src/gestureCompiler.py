import rospy
from std_msgs.msg import Int32

current_scope = ["Idle"] # will be used as a stack to push and pop scopes.
						 # Possible scopes:	"Idle", "Recording", "Number", "Repeating"
legal_gestures_in_scope = {	# Each scope is mapped to a dictionary containing gestures allowed after the previous input
	"Idle":			idle_legal_gestures,
	"Recording":	recording_legal_gestures,
	"Number":		number_legal_gestures,
	"Repeating":	repeating_legal_gestures,
	}
	
idle_legal_gestures = {
	10: [11,14],	# 10 is the run command which takes a number (which previously recorded macro to run)  as a parameter
	11: [10,12],	# 11 is the end scope/ cancel gesture (note: one frequently will have entered a scope from just ending a previous scope)
	}	# 21 is No op which if recognized, will be do nothing, but is mentioned here for completeness
recording_legal_gestures = {
	11: [11,12,13,16,17,18,19,20],	# 11 is the end scope/ cancel gesture (note: one frequently will have entered a scope from just ending a previous scope)
	12:	[11,14],	# 12 is the record macro command. The current scope is set to Recording and the next expected value should be a digit
	13:	[11,14],	# 13 is the call macro commmand. The next expected value(s) should be a digit(s) (which previously recorded macro to call)
	16:	[11,14],	# 16 is the repeat command which starts a Repeating scope and takes first a number parameter n, then an arbitrary number of actions to repeat x times. 
					# (If n is negative, nothing happens)
	17:	[11,14],	# 17 is move in the x direction (left is negative, right is positive). Expects a number (of decimeters) to move next
	18:	[11,14],	# 18 is move in the y direction (backward is negative, forward is positive)). Expects a number (of decimeters) to move next
	19:	[11,14],	# 19 is move in the z direction (down is negative, up is positive). Expects a number (of decimeters) to move next
	20:	[11,14],	# 20 is the wait command. (Expects a number (of seconds) to wait next
	}	# 21 is No op which if recognized, will be do nothing, but is mentioned here for completeness
number_legal_gestures = {
	0: 	[11,14],	# 0 is the digit 0
	1: 	[11,14],	# 1 is the digit 1
	2: 	[11,14],	# 2 is the digit 2
	3: 	[11,14],	# 3 is the digit 3
	4: 	[11,14],	# 4 is the digit 4
	5: 	[11,14],	# 5 is the digit 5
	6: 	[11,14],	# 6 is the digit 6
	7: 	[11,14],	# 7 is the digit 7
	8:	[11,14],	# 8 is the digit 8
	9:	[11,14],	# 9 is the digit 9
	14:	[0,1,2,3,4,5,6,7,8,9,15],	# 14 is the digit/number command. The next value should be the next digit in the number being constructed. Starts a Number scope if not in one.
	15:	[0,1,2,3,4,5,6,7,8,9],	# 15 is negation of a digit. (In theory, this should only be the first 'digit', but can be used before any digit in a number and the parity of the 
	}
repeating_legal_gestures = {	
	11: [11,12,13,16,17,18,19,20],	# 11 is the end scope/ cancel gesture (note: one frequently will have entered a scope from just ending a previous scope)
	12:	[11,14],	# 12 is the record macro command. The current scope is set to Recording and the next expected value should be a digit
	13:	[11,14],	# 13 is the call macro commmand. The next expected value(s) should be a digit(s) (which previously recorded macro to call)
	16:	[11,14],	# 16 is the repeat command which starts a Repeating scope and takes first a number parameter n, then an arbitrary number of actions to repeat x times. 
					# (If n is negative, nothing happens)
	17:	[11,14],	# 17 is move in the x direction (left is negative, right is positive). Expects a number (of decimeters) to move next
	18:	[11,14],	# 18 is move in the y direction (backward is negative, forward is positive)). Expects a number (of decimeters) to move next
	19:	[11,14],	# 19 is move in the z direction (down is negative, up is positive). Expects a number (of decimeters) to move next
	20:	[11,14],	# 20 is the wait command. (Expects a number (of seconds) to wait next
	}
def getLegalGestures():
	global previous_gesture, current_scope
	legal_gestures = legal_gestures_in_scope.get[current_scope]
	return legal_gestures[previous_gesture]


def getGesture():
        rospy.init_node('crazyFrog/gesture_to_macro_translator', anonymous=True)
        sub = rospy.Subscriber('crazyFrog/current_gesture', Int32 , callback)
        rospy.spin()
if __name__ == '__main__':
    try:
        getGesture()
    except rospy.ROSInterruptException:
        pass
        
#########
#
#	Write publisher (currentProgramNumber)
#
#########


def callback(gesture):
	current_gesture = gesture.data
	legal_gestures = getLegalGestures()
	
	
	
	
	
	
def possibleNextGestures(gesture):
	

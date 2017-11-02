import rospy
from std_msgs.msg import Int32
from numpy import dot

# Index of Gestures
numbers = [0,1,2,3,4,5,6,7,8,9,14,15]
Run = 10
END = 11
RecordMacro = 12
CallMacro = 13
Digit = 14
Negate = 15
Repeat = 16
MoveX = 17
MoveY = 18
MoveZ = 19
Wait = 20
NOP = 21

previous_gesture = 11
current_macro_stack = []
scope_blocks = []	# content of the scope
current_block = []	# in progress construction of Block class  [String of type, number], type can be any of Run, Call, MoveX, MoveY, MoveZ, Wait
digits = [1]
scope = ["Idle"] # will be used as a stack to push and pop scopes.
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
	global previous_gesture
	global current_scope
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
	global scope
	current_gesture = gesture.data
	if current_gesture == NOP:	# 21 is a No op
		return
	legal_gestures = getLegalGestures()
	if current_gesture not in legal_gestures:
		return
	if current_gesture in numbers
		numberHandler(current_gesture)
	if current_gesture == RUN:
	if current_gesture == END:
		endHandler()
	if current_gesture == RECORD:
		scope.append("Recording")
		# something with current macro stack/block
	
	# Add stuff here!


##############################
#
#	NOT DONE AT ALL
#	Takes care of what happens when a scope ends or an action is cancelled.
#	and alters the current number in memory, "digits", accordingly
#
###############################		
def endHandler():
	global scope
	global digits
	global previous_gesture
	global scope_blocks
	global current_block
	if previous_gesture in numbers
		scope.pop()
		num = digitsToNumber()
		if scope[-1] == "Recording":
			# FILL THIS IN
			# check the current block
		if scope[-1] == "Repeating":
			# FILL THIS IN, etc.
	# other cases too!		
	if previous_gesture in [12,16]: # cancelling record/repeat command and ends the scope they started
		scope.pop()
	# elif: # previous_gesture in [10,13,17,18,19,20]: # cancels a run/macro call/movement/wait command
	previous_gesture = END	
##############################
#
#	takes in a gesture of either place value, *digit*, or negation, 
#	and alters the current number in memory, "digits", accordingly
#
###############################
def numberHandler(gesture):
	global scope
	global digits
	if (gesture == Digit) and (scope[-1] != "Number"):	# Handle Digit if necessary
		scope.append("Number")	# If this is a new number, add a number scope and initialize the digit list. If it's not new, we can move on to the next gesture.
		digits = [1]			# 0th element of digit list is multiplier for the remainder of the digits. It is always +1 or -1.								
	elif gesture == Negate:		# Handle 'negation'
		digits[0] == -1*digits[0] # swaps sign on current_digits
	else:	# Handle when gesture is a place value for a digit (0-9)
		digits.append(gesture)
	previous_gesture = gesture	# update previous gesture
	return
	
##############################
#
#	Converts digits to the actual number it represents
#	-There's probably a more elegant way to do this.  
#   -One alternative is to store the digits in a string and concatenate, 
#	 then convert the string directly to an integer and multiply by the sign...
#
###############################
def digitsToNumber():	
	global digits
	sign = digits[0]
	length = len(digits)-1
	if length not 0:
		powers = [10**(length-n-1) for n in range(0,length)]
		num = sign *sum([digits[n]*powers[n-1] for n in range(1,length+1)])
		return num
	else:
		return 0
	
		
		
		
		
		
		
		
		
		
		
		
		
		
	

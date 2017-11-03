import rospy
import csv
import os
from std_msgs.msg import Int32
from gestureClasses import *
from gestureInfo import *

# Folder to record macros to and call macros from
macro_folder = "../macros/"
file_ext = ".csv"

previous_gesture = END
record_stack = []	# Stack of ScopeBlocks the top of which is scope in which things are currently being recorded when in a RecordMacro scope
repeat_stack = []	# Stack of ScopeBlocks the top of which is scope in which things are currently being recorded when in a Repeat scope
current_action = None	# in progress construction of an ActionBlock, type can be any in the set {"CallMacro", "MoveX", "MoveY", "MoveZ", "Wait"}
digits = [1]	# current number in progress
scope = ["Idle"] # will be used as a stack to push and pop scopes.
						 # Possible scopes:	"Idle", RecordMacro, Repeat

def getGesture():
        rospy.init_node('crazyFrog/gesture_to_macro_translator', anonymous=True)
        sub = rospy.Subscriber('crazyFrog/current_gesture', Int32 , processGesture)
        rospy.spin()
if __name__ == '__main__':
    try:
        getGesture()
    except rospy.ROSInterruptException:
        pass
        
#########
#
#	TO DO: Write publisher: runMacro(num)
#
#########

#########
#
#	TO DO: Write writeRecordedMacro(record_block)
#
#########

#########
#
#	TO DO: Write ScopeBlock class
#
#########

def getLegalGestures():
	global previous_gesture
	global scope
	legal_gestures = legal_gestures_in_scope[scope[-1]] # gets dictionary of legal gestures for current scope
	return legal_gestures[previous_gesture]				# returns from that dictionary gestures legal at this moment


##############################
#
#	processGesture(gesture)
#
#	Takes the appropriate action when a gesture is recieved from the leap motion.
#
###############################		
def processGesture(gesture):
	global scope
	current_gesture = gesture.data
	if current_gesture == NOP:	# 21 is a No op
		return
	else:
		legal_gestures = getLegalGestures()
		if current_gesture not in legal_gestures:
			return
	if current_gesture in numbers:
		numberHandler(current_gesture)
	elif current_gesture in actions:
		currentAction = ActionBlock(current_gesture)
	elif current_gesture is END:
		endHandler()
	elif current_gesture is RecordMacro:
		scope.append(RecordMacro)
	elif current_gesture is Repeat:
		scope.append(Repeat)


##############################
#
#	endHandler()
#
#	Takes care of what happens when a scope ends or an action is cancelled.
#	and alters the current number in memory, "digits", accordingly
#
###############################		
def endHandler():
	global scope
	global digits
	global previous_gesture
	global record_stack
	global repeat_stack
	global current_action
	if previous_gesture in numbers:
		scope.pop()
		num = digitsToNumber()
		if current_action is not None:
			successful = False
			try:
				current_action.setNumber(num)
				successful = True
			except MacroCallError as err:
				# this should only happen if a called macro does not exist
				print(err)
				# in this branch successful remains False because the action of calling a non-existant macro is impossible
			except ZeroActionException:
				# in this branch successful remains False because the action was to do nothing, so nothing is added to the repeat/record scopes
				pass
			# Now we add the completed action if successful
			if scope[-1] is Repeat and successful:
				repeat_stack[-1].addActionList([current_action])
			elif scope[-1] is RecordMacro and successful:
				record_stack[-1].addActionList([current_action])
			current_action = None
		else: 
		# Current action was None
			if scope[-1] is "Idle":
			# the only way to get to Idle from numbers is to finish a run call
			# so this publishes which macro to run if it exists
				try:
					runMacro(num)
				except MacroCallError as err:
					print(err)
			elif scope[-1] is Repeat:
			# the  only way to arrive here is assigning a number of times to repeat
				if num > 0:
					repeat_stack.append(ScopeBlock(num))
				else:
				# do nothing if we repeat <=  0 times
					repeat_stack.pop()
					scope.pop()
			elif scope[-1] is RecordMacro:
			# the  only way to arrive here is assigning a number to a macro being recorded
				record_stack.append(ScopeBlock(num))
				
	# previous gesture not number		
	elif previous_gesture in [RecordMacro, Repeat]: 
	# cancelling record/repeat command and ends the scope they started
		scope.pop()
	elif previous_gesture in [Run, CallMacro, MoveX, MoveY, MoveZ, Wait]: 
	# cancels a run/macro call/movement/wait command
		current_action = None
	elif previous_gesture is END:
		if scope[-1] is Repeat:
			repeat_block = repeat_stack.pop()
			num = repeat_block.getNumber()
			scope.pop()
			current_scope = scope[-1]
			if current_scope is Repeat:
				stack = repeat_stack
			else:
				stack = record_stack
			for n in range(num):
				stack[-1].addActionList(repeat_block.getActionList())	# repeats the actions in the repeat block the appropriate # of times
		if scope[-1] is RecordMacro:
			record_block = record_stack.pop()
			writeRecordedMacro(record_block)
			scope.pop()
	else:
		print("End handler is missing something. previous_gesture was "+str(previous_gesture))
	previous_gesture = END	
	return
	
	
##############################
#
#	numberHandler(gesture)
#
#	takes in a gesture of either place value, *digit*, or negation, 
#	and alters the current number in memory, "digits", accordingly
#
###############################
def numberHandler(gesture):
	global scope
	global digits
	global numbers
	global previous_gesture
	if (gesture is Digit) and (previous_gesture not in numbers):	
	# Handle Digit if necessary
		# 0th element of digits is multiplier for the remainder of the digits. 
		# It is always +1 or -1. Initialize as 1
		digits = [1]			  							
	elif gesture is Negate:	
		digits[0] == -1*digits[0] # swaps sign on current_digits
	else:	
	# Handle when gesture is a place value for a digit (0-9)
		digits.append(gesture)
	previous_gesture = gesture
	return


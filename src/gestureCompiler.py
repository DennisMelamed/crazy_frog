#!/usr/bin/env python

import rospy

import csv
from std_msgs.msg import Int32
from std_msgs.msg import String
from crazy_frog.msg import *
from gestureClasses import *
from gestureInfo import *


previous_gesture = END
current_action = None	# in progress construction of an ActionBlock, type can be any in the set {"CallMacro", "MoveX", "MoveY", "MoveZ", "Wait"}
digits = [1]		# current number in progress
scopes = ["Idle"] 	# will be used as a stack to push and pop ScopeBlock objects. "Idle" is a parent node that should never be popped/
					# Possible scopes:	"Idle", RecordMacro, Repeat

program_counter = 0
most_recent_number_run = -1


        
#########
#
#	runMacro(num)
#
#	updates the program_counter and number to broadcast if that macro number exists
#
#########
def runMacro(num):
	global program_counter
	global most_recent_number_run
	macro_file = macro_folder + "macro" + str(num)+ file_ext
	if os.path.exists(macro_file):
		program_counter = program_counter+1
		most_recent_number_run = num
	else:
		raise MacroCallError('The macro '+str(num)+' does not exist and cannot be called.')
		
#########
#
#	writeRecordedMacroToCSV(record_block)
#	
#	records a completed macro with number parameter <n> to macro<n>.<file_ext> in the macro_folder
#	file_ext and macro_folder are specified in gestureInfo.py
#
#########
def writeRecordedMacroToCSV(record_block):
	num = record_block.getNumber()
	action_string_list = record_block.stringifyMeCapN()
	#print('---WRITINING <'+ ', '.join(action_string_list) +'> to macro number '+str(num))
	with open(macro_folder + "macro" + str(num)+ file_ext, 'wb') as csv_file:
			macroWriter = csv.writer(csv_file, delimiter='\n')
			macroWriter.writerow(action_string_list)
    
##############
#
#	returns which gestures are allowed from the current state
#
##############	
def getLegalGestures():
	global previous_gesture
	global scopes
	if scopes[-1] is "Idle":
		return idle_legal_gestures[previous_gesture]
	else:
		legal_gestures = legal_gestures_in_scope[scopes[-1].getScopeType()] # gets dictionary of legal gestures for current scope
		return legal_gestures[previous_gesture]				# returns from that dictionary gestures legal at this moment





##############################
#
#	endHandler()
#
#	Takes care of what happens when a scope ends or an action is cancelled.
#	and alters the current number in memory, "digits", accordingly
#
###############################		
def endHandler():
	global scopes
	global digits
	global previous_gesture
	global current_action
	global program_counter
	global most_recent_number_run
	if previous_gesture in numbers:
		num = digitsToNumber()
		digits = [1]
		if current_action is not None and scopes[-1] is not "Idle":
			actionSuccessful = False
			try:
				current_action.setNumber(num)
				actionSuccessful = True
			except MacroCallError as err:
				# this should only happen if a called macro does not exist
				print(err)
				# in this branch successful remains False because the action of calling a non-existant macro is impossible
			except ZeroActionException:
				# in this branch successful remains False because the action was to do nothing, so nothing is added to the repeat/record scopes
				pass
			# Now we add the completed action if successful
			if actionSuccessful and scopes[-1].getScopeType() in [Repeat,RecordMacro]:
				scopes[-1].addActionList(current_action)
			current_action = None
		else: 
		# Current action was None
			if scopes[-1] is "Idle":
				# the only way to get to Idle from numbers is to finish a run call
				# so this publishes which macro to run if it exists
				try:
					runMacro(num)
				except MacroCallError as err:
					print(err)
			elif scopes[-1].getScopeType() is Repeat and num <= 0:
				# do nothing if we repeat <=  0 times
				# so remove the repeat scope from stack
				scopes.pop()
			else:
				# the  only way to arrive here is assigning a valid number of times to repeat or number to macro for recording
				scopes[-1].setNumber(num)
				
	# previous gesture not number		
	elif previous_gesture in [RecordMacro, Repeat]: 
	# cancelling record/repeat command and ends the scope they started
		scopes.pop()
	elif previous_gesture in actions or previous_gesture is Run: 
	# cancels a run/macro call/movement/wait command
		current_action = None
	elif previous_gesture is END:
		# we cannot be in an idle scope here since only ways to do that are ending from an idle number or Run, both of which are handled already
		# The only other options are Repeat or RecordMacro
		if scopes[-1].getScopeType() is Repeat:
			# we ended a repeat block, so let's add its action list repeated the appropriate # of times to the next scope
			repeat_block = scopes.pop()
			num = repeat_block.getNumber()
			for n in range(num):
				scopes[-1].addActionList(repeat_block.getActionList())	# repeats the actions in the repeat block the appropriate # of times
		else:
		# we ended a macro recording, so let's write it to a file
			record_block = scopes.pop()
			writeRecordedMacroToCSV(record_block)
	else:
		print("End handler is missing something. previous_gesture was "+str(previous_gesture))

	
	
##############################
#
#	numberHandler(gesture)
#
#	takes in a gesture of either place value, *digit*, or negation, 
#	and alters the current number in memory, "digits", accordingly
#
###############################
def numberHandler(gesture):
	global scopes
	global digits
	global numbers
	global previous_gesture
	if (gesture is Digit):
		if previous_gesture not in numbers:	
			# Handle Digit if necessary
			# 0th element of digits is multiplier for the remainder of the digits. 
			# It is always +1 or -1. Initialize as 1
			digits = [1]			  							
	elif gesture is Negate:	
		digits[0] == -1*digits[0] # swaps sign on current_digits
	elif previous_gesture in numbers[:10]
	# Replace current digit with newest number
		digits[-1] = gesture
	else:	
	# Handle when gesture is a place value for a digit (0-9)
		digits.append(gesture)
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
	if length is not 0:
		powers = [10**(length-n-1) for n in range(0,length)]
		num = sign *sum([digits[n]*powers[n-1] for n in range(1,length+1)])
		return num
	else:
		return 0

##############################
#
#	processGesture(gesture)
#
#	Takes the appropriate action when a gesture is recieved from the leap motion.
#	ENDs and numbers are handled in seperate helper functions because they are more complicated.  
#	endHandler() contains the nuts and bolts of the processing
#
###############################		
def processGesture(gesture):
	global scopes
	global previous_gesture
	global current_action
	current_gesture = gesture.data
	if current_gesture not in gestures:	# if not a recognized action, do nothing
		return
	else:
		legal_gestures = getLegalGestures()
		if current_gesture not in legal_gestures:
			return
	if current_gesture in numbers:
		numberHandler(current_gesture)
	elif current_gesture in actions:
		current_action = ActionBlock(current_gesture)
	elif current_gesture is END:
		endHandler()
	elif current_gesture is RecordMacro:
		scopes.append(ScopeBlock(RecordMacro))
	elif current_gesture is Repeat:
		scopes.append(ScopeBlock(Repeat))
	previous_gesture = current_gesture


        
def broadcastData():
    rospy.init_node('compiler_macro_publisher', anonymous=True)
    pub1 = rospy.Publisher('crazyFrog/current_program', MacroRequest, queue_size=10)
    pub2 = rospy.Publisher('crazyFrog/compiler_data', CompilerData, queue_size=10)
    sub = rospy.Subscriber('crazyFrog/current_gesture', Int32 , processGesture)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown(): # and program_counter > 0
		# Publish macro run request
		# if program_counter = 0, then nothing should be run.
    	macro_request = MacroRequest()
    	macro_request.program_counter = program_counter
    	macro_request.macro_number = most_recent_number_run
        rospy.loginfo(macro_request)
        pub1.publish(macro_request)
        # Publish compiler data for GUI (or whatever else)
        compiler_data = CompilerData()
        compiler_data.current_number = digitsToNumber()
    	compiler_data.previous_gesture = previous_gesture
    	compiler_data.current_action_block = str(current_action)
    	compiler_data.scopes = ', '.join([str(scope) for scope in scopes])
        rospy.loginfo(compiler_data)
        pub2.publish(compiler_data)
        rate.sleep()
	
if __name__ == '__main__':
    try:
        broadcastData()
    except rospy.ROSInterruptException:
        pass

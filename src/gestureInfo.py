from numpy import dot

macro_folder = "../macros/"
file_ext = ".csv"

# Index of Gestures
gestures = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
Digit = 14
Negate = 15
numbers = [0,1,2,3,4,5,6,7,8,9,Digit,Negate]
Run = 10
END = 11
RecordMacro = 12
CallMacro = 13
Repeat = 16
MoveX = 17
MoveY = 18
MoveZ = 19
Wait = 20
actions = [CallMacro, MoveX, MoveY, MoveZ, Wait]
NOP = 21
	
idle_legal_gestures = {
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
	14:	[0,1,2,3,4,5,6,7,8,9,15],	# 14 is the digit/number command. The next value should be the next digit in the number being constructed.
	15:	[0,1,2,3,4,5,6,7,8,9],	# 15 is negation of a digit.  (Can be used before any digit in a number to change the sign)  
	10: [11,14],	# 10 is the run command which takes a number (which previously recorded macro to run)  as a parameter
	11: [10,12],	# 11 is the end scope/ cancel gesture (note: one frequently will have entered a scope from just ending a previous scope)
	}	# 21 is No op which if recognized, will be do nothing, but is mentioned here for completeness
recording_legal_gestures = {
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
	14:	[0,1,2,3,4,5,6,7,8,9,15],	# 14 is the digit/number command. The next value should be the next digit in the number being constructed.
	15:	[0,1,2,3,4,5,6,7,8,9],	# 15 is negation of a digit. (Can be used before any digit in a number to change the sign) 
	11: [11,12,13,16,17,18,19,20],	# 11 is the end scope/ cancel gesture (note: one frequently will have entered a scope from just ending a previous scope)
	12:	[11,14],	# 12 is the record macro command. The current scope is set to Recording and the next expected value should be a digit
	13:	[11,14],	# 13 is the call macro commmand. The next expected value(s) should be a digit(s) (which previously recorded macro to call)
	16:	[11,14],	# 16 is the repeat command which starts a Repeating scope and takes first a number parameter n, then an arbitrary number of actions to repeat x times. 
					# (If n is 0 or negative, nothing happens)
	17:	[11,14],	# 17 is move in the x direction (left is negative, right is positive). Expects a number (of decimeters) to move next
	18:	[11,14],	# 18 is move in the y direction (backward is negative, forward is positive)). Expects a number (of decimeters) to move next
	19:	[11,14],	# 19 is move in the z direction (down is negative, up is positive). Expects a number (of decimeters) to move next
	20:	[11,14],	# 20 is the wait command. (Expects a number (of seconds) to wait next
	}	# 21 is No op which if recognized, will be do nothing, but is mentioned here for completeness
repeating_legal_gestures = {	
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
	14:	[0,1,2,3,4,5,6,7,8,9,15],	# 14 is the digit/number command. The next value should be the next digit in the number being constructed. 
	15:	[0,1,2,3,4,5,6,7,8,9],	# 15 is negation of a digit.  (Can be used before any digit in a number to change the sign) 
	11: [11,12,13,16,17,18,19,20],	# 11 is the end scope/ cancel gesture (note: one frequently will have entered a scope from just ending a previous scope)
	12:	[11,14],	# 12 is the record macro command. The current scope is set to Recording and the next expected value should be a digit
	13:	[11,14],	# 13 is the call macro commmand. The next expected value(s) should be a digit(s) (which previously recorded macro to call)
	16:	[11,14],	# 16 is the repeat command which starts a Repeating scope and takes first a number parameter n, then an arbitrary number of actions to repeat x times. 
					# (If n is 0 or negative, nothing happens)
	17:	[11,14],	# 17 is move in the x direction (left is negative, right is positive). Expects a number (of decimeters) to move next
	18:	[11,14],	# 18 is move in the y direction (backward is negative, forward is positive)). Expects a number (of decimeters) to move next
	19:	[11,14],	# 19 is move in the z direction (down is negative, up is positive). Expects a number (of decimeters) to move next
	20:	[11,14],	# 20 is the wait command. (Expects a number (of seconds) to wait next
	}	
legal_gestures_in_scope = {	# Each scope is mapped to a dictionary containing gestures allowed after the previous input
	"Idle":			idle_legal_gestures,
	RecordMacro:	recording_legal_gestures,
	Repeat:	repeating_legal_gestures,
	}

import csv
import os
from gestureInfo import *

###############################################################################
#	
#	Action Block: 
#
#	contains an action (any command that just takes a number as an argument)
#	as well as its argument number
#	can be printed to a string usable in a macro/repeat
#
###############################################################################
class ActionBlock:
	def __init__(self, action_type ,number=NaN):
		self.action_type = action_type
		self.number = number
	
	def getNumber(self, num):
		return self.number
		
	# set the number value for this action
	def setNumber(self, num):
		if action_type is not CallMacro:
			if num == 0:
				raise ZeroActionException('Ignore action')
			else:
				self.number = num
			return
		# If we reach here, we're calling a macro, so that macro better exist. Let's check on that...
		macro_file = macro_folder + "macro" + str(num)+ file_ext
		if os.path.exists(macro_file):
			self.number = num
		else:
			raise MacroCallError('The macro '+str(num)+' does not exist and cannot be called.')
		return
		
	# outputs a string for an action, or the string from a called macro
	def stringifyMeCapN():
		if self.action_type is MoveX:
			return 'x(' + str(number)+')\n'
		elif self.action_type is MoveY:
			return 'y(' + str(number)+')\n'
		elif self.action_type is MoveZ:
			return 'y(' + str(number)+')\n'
		elif self.action_type is Wait:
			return 'w(' + str(number)+')\n'
		elif self.action_type is CallMacro:
			action_string = ""
			csv.register_dialect('recorded_macro', delimiter='\n', quoting=csv.QUOTE_MINIMAL)
			with open(macro_folder + 'macro' + str(num) + file_ext, 'rb') as macro_file:
    				macro = csv.reader(macro_file, 'recorded_macro')
    				for action in macro:
    					action_string = action_string + str(action)+'\n'
			return action_string
		else:
			return ''
		
	
		
		
class MacroCallError(Exception):
     def __init__(self, value):
	     self.value = value
     def __str__(self):
         return repr(self.value)
         
class ZeroActionException(Exception):
     def __init__(self,value):
	     self.value = value
     def __str__(self):
         return repr(self.value)
         

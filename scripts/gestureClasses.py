import csv
import os
from gestureInfo import *

###############################################################################
#	
#	ActionBlock: 
#
#	contains an action (any command that just takes a number as an argument)
#	as well as its argument number
#	can be printed to a string usable in a macro/repeat
#
###############################################################################
class ActionBlock:

	def __init__(self, action_type):
		self.action_type = action_type
		self.number = 0
	
	def getActionType(self):
		return self.action_type
		
	def getNumber(self):
		return self.number
		
	# set the number value for this action
	def setNumber(self, num):
		if self.action_type is not CallMacro:
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
	def stringifyMeCapN(self):
		if self.action_type is MoveX:
			action_string= 'x(' + str(self.number)+')'
		elif self.action_type is MoveY:
			action_string= 'y(' + str(self.number)+')'
		elif self.action_type is MoveZ:
			action_string= 'z(' + str(self.number)+')'
		elif self.action_type is Wait:
			action_string= 'w(' + str(self.number)+')'
		elif self.action_type is CallMacro:
			try:
				action_string = ""
				#csv.register_dialect('recorded_macro', delimiter='\n', quoting=csv.QUOTE_NONE)
				with open(macro_folder + 'macro' + str(self.number) + file_ext, 'rb') as macro_file:
						macro = csv.reader(macro_file)#, 'recorded_macro')
						for action in macro:
							action_string = action_string + str(action[0])
			except Exception as e:
			# this should only occur if a macro file being called was deleted by the user after it was recorded, but before it was called
				print(e)	
		else:
			action_string= ''
		return action_string
			
	def __str__(self):
		if self.action_type is MoveX:
			description= 'x(' + str(self.number)+')'
		elif self.action_type is MoveY:
			description= 'y(' + str(self.number)+')'
		elif self.action_type is MoveZ:
			description= 'z(' + str(self.number)+')'
		elif self.action_type is Wait:
			description= 'w(' + str(self.number)+')'
		elif self.action_type is CallMacro:
			description= 'Call macro ' + str(self.number)
		return description

###############################################################################
#	
#	ScopeBlock: 
#
#	scope_type: 	Repeat or RecordMacro 
#	number:			number of times to repeat or number macro being recorded
#	action_list:	list of ActionBlocks which can be added to or converted to a string for export to a csv when writing macros
#
###############################################################################
class ScopeBlock:
	def __init__(self, scope_type):
		self.scope_type = scope_type
		self.action_list = []
		self.number = -1
		
	def getScopeType(self):
		return self.scope_type
		
	def getActionList(self):
		return self.action_list
		
	def getNumber(self):
		return self.number
		
	# set the number value for this action
	def setNumber(self, num):
		if self.scope_type is Repeat:
			if num <= 0:
				raise ZeroActionException('Ignore action')
			else:
				pass
		self.number = num
		
	# adds actions to the action list
	def addActionList(self, actions):
		if type(actions) is not list:
			actions = [actions]
		self.action_list = self.action_list + actions
		
	# outputs a string for each action, or the string from a called macro
	def stringifyMeCapN(self):
		if len(self.action_list):
			#print('Stringifying ' +self.__str__()+ ' <'+ ', '.join([str(act) for act in self.action_list])+'>')
			return [act.stringifyMeCapN() for act in self.action_list]#reduce(
				#(lambda x,y: x+y),					# concatenates
				#[act.stringifyMeCapN() for act in self.action_list],'')	# strings from each action
		else:
			raise(ZeroActionException('!!!empty action_list in ScopeBlock being stringified!!!!'))
	
	def __str__(self):
		if self.scope_type is RecordMacro:
			description = "Record Scope for macro " + str(self.number)
		else:
			description = "Repeat Scope to repeat " + str(self.number) + " times"
		return description
		
		
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
         

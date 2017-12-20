#!/usr/bin/env python
#
#Author: Dennis Melamed
#
import rospy
import rospkg
from tf import TransformListener
from Tkinter import *
from std_msgs.msg import *
from crazy_frog.msg import *
from geometry_msgs.msg import *
from gestureInfo import *
from os import listdir
from os.path import isfile, join

current_scopes = ''
current_scope = ''
current_action_block = ''
current_number = -1
previous_gestures = ['NOP']
current_gesture = 21
legal_gestures = []
current_pos = ""
goal_pos = ""
current_command = ""
command_queue = ""

r = rospkg.RosPack()

macro_path = r.get_path('crazy_frog') + "/macros/"

gesture_friendly_names = ['0',
                           '1',
                           '2',
                           '3',
                           '4',
                           '5',
                           '6',
                           '7',
                           '8',
                           '9',
                           'Run',
                           'END',
                           'RecordMacro',
                           'CallMacro',
                           'Digit',
                           'Negate',
                           'Repeat',
                           'MoveX',
                           'MoveY',
                           'MoveZ',
                           'Wait',
                           'NOP',
                           'SetNumberVar']


	

#get the info the compiler is publishing, like the scopes and gestures and macro numbers
def compilerCallback(data):
	global current_scopes
        global current_scope
	global current_action_block
	global current_number
	global previous_gestures
	global legal_gestures
	current_scopes = data.scopes
        current_scope = data.current_scope.split(",")
	current_action_block = data.current_action_block
	current_number = data.current_number
	if previous_gestures[-1] != gesture_friendly_names[data.previous_gesture]:
		previous_gestures.append(gesture_friendly_names[data.previous_gesture])
        legal_gestures = str([gesture_friendly_names[int(x)] for x in data.legal_gestures.split(",")])

#get goal positino data from the the runtime
def runtimeCallback(data):
	global goal_pos
	goal_pos = "[" + str(data.pose.position.x) + ", "+ str(data.pose.position.y) + ", "+str(data.pose.position.z) + "]"
#get the current gesture from the gesture recognizer node
def gestureCallback(data):
	global current_gesture
	current_gesture = data.data
#the runtime publishes the current command being run, and the command queue. collect them in these callbacks
def currentCommandCallback(data):
	global current_command
	current_command = data.data

def commandQueueCallback(data):
	global command_queue
	command_queue = data.data

#displays the desired macro file contents in the macro display window
def display_text(file_path, text_label):
	global macro_path
	#print file_path
	text_label.set("")
	full_path = macro_path + file_path
	with open(full_path) as f:
		text_label.set(f.read())

#main chunk
#this is a very bloated method, most of the setup will be moved to a seperate method
#the updating could also be moved out, for debugging this is the most useful way to have it currently
def talker():
	global current_scopes	
	global current_action_block
	global current_number
	global previous_gestures
	global current_gesture	
	global legal_gestures
	global current_pos
	global goal_pos
	global current_command
	global command_queue

        #listen to everyone to get some info to display
        pub = rospy.Publisher('info', String, queue_size=10)
	compiler = rospy.Subscriber('crazyFrog/compiler_data', CompilerData, compilerCallback)
	runtime_goal = rospy.Subscriber('goal', PoseStamped, runtimeCallback)
	gesture = rospy.Subscriber('crazyFrog/current_gesture', Int32, gestureCallback)
	runtime_current_command = rospy.Subscriber('crazyFrog/current_command', String, currentCommandCallback)
	runtime_command_queue   = rospy.Subscriber('crazyFrog/command_queue', String, commandQueueCallback)
	
        #handles getting the current UAV transform
	tf = TransformListener()

        rospy.init_node('gui', anonymous=True)
        rate = rospy.Rate(10) # 10hz
	
	top = Tk()

	#######################################
	#OVERALL FORMATTING
	#######################################

	#FIRST ROW
	m1 = PanedWindow(orient=HORIZONTAL)
	m1.pack(fill=BOTH, expand=1)
	m1.config(showhandle=1, sashrelief=RAISED)

	#SECOND ROW
	m2 = PanedWindow(orient=HORIZONTAL)
        m2.pack(fill=BOTH, expand=1)
        m2.config(showhandle=1, sashrelief=RAISED)	

	#THIRD ROW
	m3 = PanedWindow(orient=HORIZONTAL)
        m3.pack(fill=BOTH, expand=1)
        m3.config(showhandle=1, sashrelief=RAISED)	
	
	#RIGHT SIDE OF WINDOW
	m4 = PanedWindow(orient=VERTICAL)
        m4.pack(fill=BOTH, expand=1)
        m4.config(showhandle=0, sashrelief=RAISED)	
	m4.add(m1)
	m4.add(m2)
	m4.add(m3)

	#DIRECTORY BAR
	m5 = PanedWindow()
	m5.pack(fill=BOTH, expand=1)
	m5.config(showhandle=0, sashrelief=RAISED)

	#MACRO DISPLAY WINDOW
	m7 = PanedWindow()
	m7.pack(fill=BOTH, expand=1)
	m7.config(showhandle=0, sashrelief=RAISED)
	

	#OVERALL STRUCTURE
	m6 = PanedWindow(orient=HORIZONTAL)
	m6.pack(fill=BOTH, expand=1)
	m6.config(showhandle=1, sashrelief=RAISED)
	m6.add(m5)
	m6.add(m7)
	m6.add(m4)

	

	##############################################
	#INDIVIDUAL PANES
	##############################################

	#ROW1
        #1,1: Unclosed Scopes
	current_scopes_tk = StringVar()
	current_scopes_tk.set(current_scopes)
	frame1 = LabelFrame(top, text='Unclosed Scopes', width=400, height=600)
	unclosed_label = Label(frame1, textvariable=current_scopes_tk)
	unclosed_label.pack()
	m1.add(frame1)

        #1,2: Current Scope
        current_scope_tk = StringVar()
        current_scope_tk.set(current_scope)
	frame2 = LabelFrame(top, text='Current Scope', width=600, height=400)
	current_scope_label = Label(frame2, textvariable=current_scope_tk)
	current_scope_label.pack()
	m1.add(frame2)
	
        #1,3: Current Action & Curretn Number being built
	current_action_block_tk = StringVar()
	current_action_block_tk.set(current_action_block)
	current_number_tk = StringVar()
	current_number_tk.set(current_number)
	frame3 = LabelFrame(top, text='Action & Current Number', width=200, height=200)
	current_action_label = Label(frame3, textvariable=current_action_block_tk)
	current_number_label = Label(frame3, textvariable=current_number_tk)
        current_action_label.pack()
	current_number_label.pack()
	m1.add(frame3)


	#ROW2
        #2,1: Previous gestures the compiler has used
	previous_gestures_tk = StringVar()
	previous_gestures_tk.set(str(previous_gestures))
	frame4 = LabelFrame(top, text='Previous Gestures', width=600, height=200)
	previous_gestures_label = Label(frame4, textvariable=previous_gestures_tk, wraplength=100)
	previous_gestures_label.pack()
	m2.add(frame4)

        #2,2: Legal next gestures given the previous gesture
	legal_gestures_tk = StringVar()
	legal_gestures_tk.set(str(legal_gestures))
	frame5 = LabelFrame(top, text='Legal Next Gestures', width=600, height=600)
	legal_gestures_label = Label(frame5, textvariable=legal_gestures_tk, wraplength = 100)
	legal_gestures_label.pack()
	m2.add(frame5)

        #2,3: Current gesture being recognized by the Leap
	current_gesture_tk = StringVar()
	current_gesture_tk.set(str(gesture_friendly_names[current_gesture]))
	frame6 = LabelFrame(top, text='Current Gesture', width=200, height=200)
	current_gesture_label = Label(frame6, textvariable=current_gesture_tk)
	current_gesture_label.pack()
	m2.add(frame6)

	
	#ROW3
        #3,1: Current command queue the runtime is working through
	command_queue_tk = StringVar()
	command_queue_tk.set(command_queue)
	frame7 = LabelFrame(top, text='UAV Command Queue', width=200, height=600)
	command_queue_label = Label(frame7, textvariable=command_queue_tk)
	command_queue_label.pack()
	m3.add(frame7)
	
        #3,2: Current command the UAV is executing
	current_command_tk = StringVar()
	frame8 = LabelFrame(top, text='Current UAV Command', width=200, height=200)
	current_command_label = Label(frame8, textvariable=current_command_tk)
	current_command_label.pack()
	m3.add(frame8)	

        #3,3: The current and goal positions of the UAV
	current_pos_tk = StringVar()
	current_pos_tk.set(str(current_pos))
	goal_pos_tk = StringVar()
	goal_pos_tk.set(str(goal_pos))
	frame9 = LabelFrame(top, text="UAV Position Information")
	current_pos_name_label = Label(frame9, text="Current Position:")
	current_pos_label = Label(frame9, textvariable=current_pos_tk)
	goal_pos_name_label = Label(frame9, text="Goal Position:")
	goal_pos_label = Label(frame9, textvariable=goal_pos_tk)
	current_pos_name_label.pack()
	current_pos_label.pack()
	goal_pos_name_label.pack()
	goal_pos_label.pack()
	m3.add(frame9)


	#VIEW THE CONTENTS OF MACRO FILES HERE
	display_text_frame = LabelFrame(top, text="Macro View", width = 200)
	file_contents_label_tk = StringVar()
	file_contents_label = Label(display_text_frame, textvariable=file_contents_label_tk)
	file_contents_label.pack()
	m7.add(display_text_frame)

        #Listing of macros that can be clicked on to display their contents (far left panel)
	frame_dir = LabelFrame(top, text="Macros", width = 200)
	x = [f for f in listdir(macro_path) if isfile(join(macro_path,f))]
        macro_label = []
	for f in x:
            macro_label.append(Button(frame_dir, text = f, command=lambda f=f: display_text(f, file_contents_label_tk)))
	    macro_label[-1].pack()
	m5.add(frame_dir)



        while not rospy.is_shutdown():
                #update labels based on most recent information
		current_scopes_tk.set(current_scopes)
                current_scope_tk.set(current_scope)
	    	current_action_block_tk.set(current_action_block)
	    	current_number_tk.set(current_number)
            	current_gesture_tk.set(str(gesture_friendly_names[current_gesture]))
	    	previous_gestures_tk.set(str(previous_gestures))
	    	legal_gestures_tk.set(str(legal_gestures))
		goal_pos_tk.set(goal_pos)
	    	current_command_tk.set(current_command)
		command_queue_tk.set(command_queue)


		#get latest UAV-world transform
	   	t = tf.getLatestCommonTime("/cam_pos","/world")
		current_pos, q = tf.lookupTransform("/cam_pos", "/world", t)
		current_pos_tk.set( "[" + format(current_pos[0], '.2f') + ", " + format(current_pos[1], '.2f') +", "+ format(current_pos[2], '.2f') + "]") 
	    
                #update the macro listing on the far left in case anything changed
		for widget in frame_dir.winfo_children():
		 	if not (widget.cget("text") in listdir(macro_path)):
		 		widget.destroy()
		
		for f in listdir(macro_path) :
			if isfile(join(macro_path,f)) and f not in x:
                            macro_label.append(Button(frame_dir, text = f, command=lambda f=f:display_text(f,file_contents_label_tk)))
			    macro_label[-1].pack()
                            x.append(f)
			

            	rate.sleep()
	    	top.update_idletasks()
	    	top.update()

if __name__ == '__main__':
    try:
	
        talker()
    except rospy.ROSInterruptException:
        pass

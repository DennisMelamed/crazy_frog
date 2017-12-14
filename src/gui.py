#!/usr/bin/env python
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
                           'NOP']

	


def compilerCallback(data):
	global current_scopes
	global current_action_block
	global current_number
	global previous_gestures
	global legal_gestures
	current_scopes = data.scopes
	current_action_block = data.current_action_block
	current_number = data.current_number
	if previous_gestures[-1] != gesture_friendly_names[data.previous_gesture]:
		previous_gestures.append(gesture_friendly_names[data.previous_gesture])
	if current_scopes.split(",")[-1] == "Idle":
		legal_gestures = [gesture_friendly_names[x] for x in legal_gestures_in_scope["Idle"][data.previous_gesture]]
	elif "Record" in current_scopes.split(',')[-1]:
		legal_gestures = [gesture_friendly_names[x] for x in legal_gestures_in_scope[12][data.previous_gesture]]
	elif "Repeat" in current_scopes.split(',')[-1]:
		legal_gestures = [gesture_friendly_names[x] for x in legal_gestures_in_scope[16][data.previous_gesture]]

def runtimeCallback(data):
	global goal_pos
	goal_pos = "[" + str(data.pose.position.x) + ", "+ str(data.pose.position.y) + ", "+str(data.pose.position.z) + "]"
def gestureCallback(data):
	global current_gesture
	current_gesture = data.data

def currentCommandCallback(data):
	global current_command
	current_command = data.data

def commandQueueCallback(data):
	global command_queue
	command_queue = data.data

def display_text(file_path, text_label):
	global macro_path
	print file_path
	text_label.set("")
	full_path = macro_path + file_path
	with open(full_path) as f:
		text_label.set(f.read())

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

        pub = rospy.Publisher('info', String, queue_size=10)
	compiler = rospy.Subscriber('crazyFrog/compiler_data', CompilerData, compilerCallback)
	runtime_goal = rospy.Subscriber('goal', PoseStamped, runtimeCallback)
	gesture = rospy.Subscriber('crazyFrog/current_gesture', Int32, gestureCallback)
	runtime_current_command = rospy.Subscriber('crazyFrog/current_command', String, currentCommandCallback)
	runtime_command_queue   = rospy.Subscriber('crazyFrog/command_queue', String, commandQueueCallback)
	

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
        m4.config(showhandle=1, sashrelief=RAISED)	
	m4.add(m1)
	m4.add(m2)
	m4.add(m3)

	#DIRECTORY BAR
	m5 = PanedWindow()
	m5.pack(fill=BOTH, expand=1)
	m5.config(showhandle=1, sashrelief=RAISED)

	#MACRO DISPLAY WINDOW
	m7 = PanedWindow()
	m7.pack(fill=BOTH, expand=1)
	m7.config(showhandle=1, sashrelief=RAISED)
	

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
	#1,1
	current_scopes_tk = StringVar()
	current_scopes_tk.set(current_scopes)
	frame1 = LabelFrame(top, text='Unclosed Scopes', width=200, height=200)
	unclosed_label = Label(frame1, textvariable=current_scopes_tk)
	unclosed_label.pack()
	m1.add(frame1)

	#1,2
	scope_queue = ""
	current_scope = "Idle"
	current_action = None
	scope_queue_tk = StringVar()
	frame2 = LabelFrame(top, text='Current Scope NOT RIGHT CURRENTLY', width=200, height=200)
	current_scope_label = Label(frame2, textvariable=scope_queue_tk)
	current_scope_label.pack()
	m1.add(frame2)
	
	#1,3
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
	#2,1
	previous_gestures_tk = StringVar()
	previous_gestures_tk.set(str(previous_gestures))
	frame4 = LabelFrame(top, text='Previous Gestures', width=200, height=200)
	previous_gestures_label = Label(frame4, textvariable=previous_gestures_tk, wraplength=100)
	previous_gestures_label.pack()
	m2.add(frame4)

	#2,2
	legal_gestures_tk = StringVar()
	legal_gestures_tk.set(str(legal_gestures))
	frame5 = LabelFrame(top, text='Legal Next Gestures', width=200, height=200)
	legal_gestures_label = Label(frame5, textvariable=legal_gestures_tk, wraplength = 100)
	legal_gestures_label.pack()
	m2.add(frame5)

	#2,3
	current_gesture_tk = StringVar()
	current_gesture_tk.set(str(gesture_friendly_names[current_gesture]))
	frame6 = LabelFrame(top, text='Current Gesture', width=200, height=200)
	current_gesture_label = Label(frame6, textvariable=current_gesture_tk)
	current_gesture_label.pack()
	m2.add(frame6)

	
	#ROW3
	#3,1
	command_queue_tk = StringVar()
	command_queue_tk.set(command_queue)
	frame7 = LabelFrame(top, text='UAV Command Queue', width=200, height=200)
	command_queue_label = Label(frame7, textvariable=command_queue_tk)
	command_queue_label.pack()
	m3.add(frame7)
	
	#3,2
	current_command_tk = StringVar()
	frame8 = LabelFrame(top, text='Current UAV Command', width=200, height=200)
	current_command_label = Label(frame8, textvariable=current_command_tk)
	current_command_label.pack()
	m3.add(frame8)	

	#3,3
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


	#DIRECTORY PANE
	display_text_frame = LabelFrame(top, text="Macro View", width = 200)
	file_contents_label_tk = StringVar()
	file_contents_label = Label(display_text_frame, textvariable=file_contents_label_tk)
	file_contents_label.pack()
	m7.add(display_text_frame)


	frame_dir = LabelFrame(top, text="Macros", width = 200)
	x = [f for f in listdir(macro_path) if isfile(join(macro_path,f))]
	for f in x:
		macro_label = Label(frame_dir, text = f)
		macro_label.bind("<Button-1>", lambda e: display_text(f, file_contents_label_tk))
		macro_label.pack()
	m5.add(frame_dir)



        while not rospy.is_shutdown():
		current_scopes_tk.set(current_scopes)
	    	current_action_block_tk.set(current_action_block)
	    	current_number_tk.set(current_number)
            	current_gesture_tk.set(str(gesture_friendly_names[current_gesture]))
	    	previous_gestures_tk.set(str(previous_gestures))
	    	legal_gestures_tk.set(str(legal_gestures))
		goal_pos_tk.set(goal_pos)
	    	current_command_tk.set(current_command)
		command_queue_tk.set(command_queue)
		
		if current_scope is not current_scopes.split(",")[-1]:
			scope_queue = ""
			current_scope = current_scopes.split(",")[-1]
		if current_action != current_action_block:
			scope_queue += str(current_action_block)
			current_action = current_action_block
		print "scope: " + str(current_scope)
		print "current_action: " + str(current_action)
		print "current_action_block: " + str(current_action_block)
		print "scope_queue: " + str(scope_queue)
		
	   	t = tf.getLatestCommonTime("/cam_pos","/world")
		current_pos, q = tf.lookupTransform("/cam_pos", "/world", t)
		current_pos_tk.set( "[" + format(current_pos[0], '.2f') + ", " + format(current_pos[1], '.2f') +", "+ format(current_pos[2], '.2f') + "]") 
	    
		for widget in frame_dir.winfo_children():
		 	if not (widget.cget("text") in listdir(macro_path)):
		 		widget.destroy()
		
		for f in listdir(macro_path) :
			if isfile(join(macro_path,f)) and f not in x:
	    			macro_label = Label(frame_dir, text = f)
				x.append(f)
				macro_label.bind("<Button-1>", lambda e: display_text(macro_label.cget("text"), file_contents_label_tk))
				macro_label.pack()
			

		rospy.loginfo(previous_gestures)
            	pub.publish(str(current_gesture))
            	rate.sleep()
	    	top.update_idletasks()
	    	top.update()

if __name__ == '__main__':
    try:
	
        talker()
    except rospy.ROSInterruptException:
        pass

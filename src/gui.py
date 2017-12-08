#!/usr/bin/env python
import rospy
from tf import TransformListener
from Tkinter import *
from std_msgs.msg import *
from crazy_frog.msg import *
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

macro_path = "../macros/"

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
	else:
		legal_gestures = [gesture_friendly_names[x] for x in legal_gestures_in_scope[previous_gestures[-1]][data.previous_gesture]]

#def runtimeCallback():
	
def gestureCallback(data):
	global current_gesture
	current_gesture = data.data

def talker():
	global current_scopes	
	global current_action_block
	global current_number
	global previous_gestures
	global current_gesture	
	global legal_gestures
	global current_pos
	global goal_pos

        pub = rospy.Publisher('info', String, queue_size=10)
	compiler = rospy.Subscriber('crazyFrog/compiler_data', CompilerData, compilerCallback)
	#runtime = rospy.Subscriber('crazyFrog/runtime_data', RuntimeData, runtimeCallback)
	gesture = rospy.Subscriber('crazyFrog/current_gesture', Int32, gestureCallback)
	
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
	

	#OVERALL STRUCTURE
	m6 = PanedWindow(orient=HORIZONTAL)
	m6.pack(fill=BOTH, expand=1)
	m6.config(showhandle=1, sashrelief=RAISED)
	m6.add(m5)
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
	frame2 = LabelFrame(top, text='Current Scope NOT RIGHT CURRENTLY', width=200, height=200)
	current_scope_label = Label(frame2, textvariable=current_scopes_tk)
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
	
	#3,2
	
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
	frame_dir = LabelFrame(top, text="Macros", width = 200)
	x = [f for f in listdir(macro_path) if isfile(join(macro_path,f))]
	for f in x:
		macro_label = Label(frame_dir, text = f)
		macro_label.pack()
	m5.add(frame_dir)


        while not rospy.is_shutdown():
	    current_scopes_tk.set(current_scopes)
	    current_action_block_tk.set(current_action_block)
	    current_number_tk.set(current_number)
            current_gesture_tk.set(str(gesture_friendly_names[current_gesture]))
	    previous_gestures_tk.set(str(previous_gestures))
	    legal_gestures_tk.set(str(legal_gestures))
	    
	    if tf.frameExists("/cam_pos") and tf.frameExists("/world"):
		t = tf.getLatestCommonTime("/cam_pos","/world")
		current_pos, q = tf.lookupTransform("/cam_pos", "/world", t)
		current_pos = str(current_pos)
	    if tf.frameExists("/goal") and tf.frameExists("/world"):
		t = tf.getLatestCommonTime("/goal", "/world")
		goal_pos,q = tf.lookupTransform("/goal", "/world")
		goal_pos = str(goal_pos)

	    
	    for f in listdir(macro_path):
		if isfile(join(macro_path,f)) and f not in x:
	    		macro_label = Label(frame_dir, text = f)
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

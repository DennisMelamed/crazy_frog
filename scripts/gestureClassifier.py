import numpy as np
from sklearn import neighbors, datasets
import pickle
import glob
import os
import time

import rospy
import pprint
from leap_motion.msg import leapros
from std_msgs.msg import Int32






gesture_class_nums = {  "0":0,#
                        "1":1,#
                        "2":2,#
                        "3":3,#
                        "4":4,#
                        "5":5,#
                        "6":6,#
                        "7":7,#
                        "8":8,#
                        "9":9,#
                        "Run":10,#
                        "END":11,#
                        "RecordMacro":12,#
                        "CallMacro":13,#
                        "Digit":14,#
                        "Negate":15,#
                        "Repeat":16,
                        "MoveX":17,
                        "MoveY":18,
                        "MoveZ":19,
                        "Wait":20,
                        "NOP":21 }#



# How many neighbors do happy villagers have?
n_neighbors = 200

# import data
file_extension = "pkl"
inp_folder = "../classification_data/"

out_folder = inp_folder + "classifier_output/"
if not os.path.exists(out_folder):
	os.mkdir(out_folder)
      
# find pickles in input folder
pickled_gestures = glob.glob(inp_folder + "*" + file_extension)

pprint.pprint(pickled_gestures)

print '--'
# format data into np.arrays 
#	gesture_data is the dataset
#	class_data is the corresponding class for each row
class_num = 0
dict_class_gesture = {}
for pickled_gesture in pickled_gestures:
        gesture_name = os.path.splitext(os.path.basename(pickled_gesture))[0]
        gesture_name = gesture_name[:-7]
        print gesture_name
	dict_class_gesture.update({class_num:pickled_gesture})
	with open(pickled_gesture, 'r+b') as f_handler:
		new_gesture_data = pickle.load(f_handler)			#	numpy array
	if class_num == 0:
                gesture_data = new_gesture_data
                pprint.pprint(gesture_data)
		class_data = np.full((1,gesture_data.shape[0]), gesture_class_nums[gesture_name])
	else:
                gesture_data = np.vstack((gesture_data, new_gesture_data)) 
		class_data = np.hstack((class_data, np.full((1,new_gesture_data.shape[0]), gesture_class_nums[gesture_name])))
                print gesture_class_nums[gesture_name]
	class_num = 1 + class_num
        print "--"
# output a dictionary of what labels correspond to what gestures
# pickle it for later use
output = open(out_folder+'class_gesture_dict.pkl', 'wb')
pickle.dump(dict_class_gesture, output)
output.close()
	
#for weights in ['uniform', 'distance']:
weights = 'distance'  
      # we create an instance of Neighbours Classifier and fit the data.
print "len of gesture Data: " +str(len(gesture_data))
print "len of gesture data[0]: " + str(len(gesture_data[0]))
print "len of class data: " + str(len(class_data))
pprint.pprint(class_data[0][20])
classy_mammer_jammer = neighbors.KNeighborsClassifier(n_neighbors, weights=weights)
classy_mammer_jammer.fit(gesture_data, class_data[0])

print classy_mammer_jammer.get_params(True)

# output the actual classifier to be later used for prediction
output = open(out_folder+'classifier.pkl', 'wb')
pickle.dump(classy_mammer_jammer, output)
output.close()

string = ''
for pickled_gesture in pickled_gestures:
    with open(pickled_gesture, 'r+b') as f_handler:
	new_gesture_data = pickle.load(f_handler)			#	numpy array
        for row in new_gesture_data:
           # print classy_mammer_jammer.predict(row.reshape(1,-1))
            if classy_mammer_jammer.predict(row.reshape(1,-1))[0] is not 0:
                string = string + str(classy_mammer_jammer.predict(row.reshape(1,-1))[0])          
print string
print "done!"






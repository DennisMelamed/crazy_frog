import numpy as np
from sklearn import neighbors, datasets
import pickle

import glob
import os
import time


gesture_class_nums = {  "0":0,
                        "1":1,
                        "2":2,
                        "3":3,
                        "4":4,
                        "5":5,
                        "6":6,
                        "7":7,
                        "8":8,
                        "9":9,
                        "run":10,
                        "end":11,
                        "record":12,
                        "call":13,
                        "digit":14,
                        "negate":15,
                        "repeat":16,
                        "x":17,
                        "y":18,
                        "z":19,
                        "wait":20,
                        "nop":21 }



# How many neighbors do happy villagers have?
n_neighbors = 5

# import data
file_extension = "pkl"
inp_folder = "../classification_data/"

out_folder = inp_folder + "classifier_output/"
if not os.path.exists(out_folder):
	os.mkdir(out_folder)
      
# find pickles in input folder
pickled_gestures = glob.glob(inp_folder + "*" + file_extension)

print pickled_gestures
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
		class_data = np.full((1,gesture_data.shape[0]),gesture_class_nums[gesture_name])
	else:
		gesture_data = np.vstack((gesture_data, new_gesture_data)) 
		class_data = np.hstack((class_data, np.full((1,new_gesture_data.shape[0]),gesture_class_nums[gesture_name])))
	class_num = 1 + class_num

# output a dictionary of what labels correspond to what gestures
# pickle it for later use
output = open(out_folder+'class_gesture_dict.pkl', 'wb')
pickle.dump(dict_class_gesture, output)
output.close()
	
#for weights in ['uniform', 'distance']:
weights = 'distance'  
      # we create an instance of Neighbours Classifier and fit the data.
print len(gesture_data)
print len(gesture_data[0])
print len(class_data)
print class_data
classy_mammer_jammer = neighbors.KNeighborsClassifier(n_neighbors, weights=weights)
classy_mammer_jammer.fit(gesture_data, class_data[0])

# output the actual classifier to be later used for prediction
output = open(out_folder+'classifier.pkl', 'wb')
pickle.dump(classy_mammer_jammer, output)
output.close()

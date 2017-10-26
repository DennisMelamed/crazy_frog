import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from sklearn import neighbors, datasets
import pickle

import glob
import os
import time

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

# format data into np.arrays 
#	gesture_data is the dataset
#	class_data is the corresponding class for each row
class_num = 0
dict_class_gesture = {}
for pickled_gesture in pickles:
	dict_class_gesture.update({class_num:pickled_gesture})
	with open(pickled_gesture, 'r+b') as f_handler:
		new_gesture_data = pickle.load(f_handler)			#	numpy array
	if class_num == 0:
		gesture_data = new_gesture_data
		class_data = np.full((1,gesture_data.shape[0]),class_num)
	else:
		gesture_data = np.vstack((gesture_data, new_gesture_data)) 
		class_data = np.hstack(class_data,np.full((1,new_gesture_data.shape[0]),class_num))
	class_num = 1 + class_num

# output a dictionary of what labels correspond to what gestures
# pickle it for later use
output = open(out_folder+'class_gesture_dict.pkl', 'wb')
pickle.dump(dict_class_gesture, output)
output.close()
	
#for weights in ['uniform', 'distance']:
weights = 'distance'  
      # we create an instance of Neighbours Classifier and fit the data.
    classy_mammer_jammer = neighbors.KNeighborsClassifier(n_neighbors, weights=weights)
    classy_mammer_jammer.fit(gesture_data, class_data)

# output the actual classifier to be later used for prediction
output = open(out_folder+'classifier.pkl', 'wb')
pickle.dump(classy_mammer_jammer, output)
output.close()

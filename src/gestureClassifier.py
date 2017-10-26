import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from sklearn import neighbors, datasets
import pickle


import glob
import os
import time

n_neighbors = 5

# import data
file_extension = "pkl"
inp_folder = ""
out_folder = ""
# find pickles in input folder
pickled_gestures = glob.glob(inp_folder + "*" + file_extension)

# format data into np.arrays 
#	gesture_data is the dataset
#	class_data is the corresponding class for each row
class_num = 0
dict_class_gesture = {}
for pickled_gesture in pickles:
	with open(pickled_gesture, 'r+b') as f_handler:
		raw_data = pickle.load(f_handler)			#	I'm assuming raw_data will give me a text file and not actual columns of data...
	dict_class_gesture.update({class_num:raw_data})
	new_gesture_data = np.loadtxt("raw_data", delimeter=",") # not sure of the data format so this may not work if above assumption is false
	if class_num == 0:
		gesture_data = new_gesture_data
		class_data = np.full((1,gesture_data.shape[0]),class_num)
	else:
		gesture_data = np.vstack((gesture_data, new_gesture_data)) 
		class_data = np.hstack(class_data,np.full((1,new_gesture_data.shape[0]),class_num))
	class_num = 1 + class_num
	

#h = .02  # step size in the mesh
#
#
## Create color maps
#cmap_light = ListedColormap(['#FFAAAA', '#AAFFAA', '#AAAAFF'])
#cmap_bold = ListedColormap(['#FF0000', '#00FF00', '#0000FF'])
#
#for weights in ['uniform', 'distance']:
weights = 'distance'  
      # we create an instance of Neighbours Classifier and fit the data.
    clf = neighbors.KNeighborsClassifier(n_neighbors, weights=weights)
    clf.fit(gesture_data, class_data)

#    # Plot the decision boundary. For that, we will assign a color to each
#    # point in the mesh [x_min, x_max]x[y_min, y_max].
#    x_min, x_max = X[:, 0].min() - 1, X[:, 0].max() + 1
#    y_min, y_max = X[:, 1].min() - 1, X[:, 1].max() + 1
#    xx, yy = np.meshgrid(np.arange(x_min, x_max, h),
#                        np.arange(y_min, y_max, h))
#   Z = clf.predict(np.c_[xx.ravel(), yy.ravel()])
#
#    # Put the result into a color plot
#    Z = Z.reshape(xx.shape)
#    plt.figure()
#    plt.pcolormesh(xx, yy, Z, cmap=cmap_light)
#
#  # Plot also the training points
#    plt.scatter(X[:, 0], X[:, 1], c=y, cmap=cmap_bold, edgecolor='k', s=20)
#    plt.xlim(xx.min(), xx.max())
#    plt.ylim(yy.min(), yy.max())
#    plt.title("3-Class classification (k = %i, weights = '%s')"% (n_neighbors, weights))
#
#plt.show()

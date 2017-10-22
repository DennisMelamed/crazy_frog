#!/usr/bin/env python
# license removed for brevity
import rospy
import pickle
import sklearn
from std_msgs.msg import String
from leap_motion.msg import leapros
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from sklearn import neighbors, datasets

numPointsToCollect = 20
n_neighbors = 2
i = 0
X = np.empty([numPointsToCollect,2], dtype=float)
y = np.empty([numPointsToCollect], dtype=int)
printflag = False

def spitOutResults():
    global printflag
    global X, y
    if not printflag:
        with open(rospy.get_param('/gesture_name') + 'RawData.pkl', 'wb') as f:
            pickle.dump(X, f)
        print X
        printflag = True


def callback(data):
    global i
    global X
    global y
    global numPointsToCollect
    if i < numPointsToCollect:
        X[i] = ([data.direction.x, data.direction.y])
        y[i] = (int(data.direction.z))
        i = i+1
    else:
        spitOutResults()

def recording():
        rospy.init_node('classifier', anonymous=True)
        sub = rospy.Subscriber('leapmotion/data', leapros , callback)
        rospy.spin()
if __name__ == '__main__':
    try:
        recording()
    except rospy.ROSInterruptException:
        pass

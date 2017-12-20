### Dependencies
* ROS Indigo/Kinetic (Hasn't been tested with other distros)
* Leap Motion SDK (Ubuntu 14.04 and 16.04 have been tested)
* Python2.7
* scikit-learn
* numpy
* tf
* crazyflie_ros node (https://github.com/whoenig/crazyflie_ros)
* http://www.coppeliarobotics.com/files/tmp/ (Download the latest version of V-REP here and go through their ROS interfacing tutorials)
* tKinter

### Goals
* Provide gestural macros for controlling microUAVs

### Current State
* Basically finished, but could use some TLC (i.e. elegance, new features)

### Nodes
* Leap to Gesture Node 
publishes gesture from known library based on programmer
* Compiler Node - Gestures to Modular commands Node
Given some set of gestures (>=1), forms a complex command, eventually writes out a file that contains simple commands for a certain macro
* Runtime Node
Takes a macro file and runs it on the crazyFlie as appropriate
* Classification Data Collector
Run while performing a gesture over the Leap Motion to train that gesture. Ensure the ROS parameter /gesture_name is set with the name of the gesture currently being recorded

### Scripts
* gestureClassifier.py
Just run this with python2.7, this will build the classifier from the gesture pickles in the classification_data directory (which you recorded with the ClassificationDataCollector ROS node)

### Launch Files
Once the classifier is built using the gestureClassifier script, running gesture_recognition.launch launches the full pipeline: the leap_motion node's sender.py, gestureRecognizer.py, gestureCompiler.py, the runtime, and gui.py.
You need to run ''sudo leapd'' prior to running the launch command to ensure the leap motion process is running. Running the command LeapControlPanel gives you access to a visualizer which can be super useful in debugging commands.

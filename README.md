### Dependencies
* ROS Indigo
* Leap Motion SDK (Ubuntu)
* scikit-learn
* numpy
* tf
* crazyflie_ros node
* http://www.coppeliarobotics.com/files/tmp/

### Goals
* Provide gestural macros for controlling microUAVs

### Current State
* In progress, most nodes have their meat finished

### Nodes
* Leap to Gesture Node 
publishes gesture from known library based on programmer
* Compiler Node - Gestures to Modular commands Node
Given some set of gestures (>=1), forms a complex command, eventually writes out a file that contains simple commands for a certain macro
* Runtime Node
Takes a macro file and runs it on the crazyFlie as appropriate

### Messages

* Macro Request
Int32 program_counter "this is the nth macro I want to add to the queue"
Int32 macro_number    "this is the number of the macro I want to add to the queue"

### Services


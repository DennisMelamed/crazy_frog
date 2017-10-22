### Dependencies
* ROS Indigo
* Leap Motion SDK (Ubuntu)

### Goals
* Provide gestural macros for controlling microUAVs

### Current State
* Non-existent

### Nodes
* Leap to Gesture Node 
publishes gesture from known library based on programmer
* Interpret Node - Gestures to Modular commands Node
Given some set of gestures (>=1), forms a complex command
* Compiler Node - builds queue of RPY+Thrust+Timing
Translates complex command into vector of RPYTT
* Run Node 
takes queue of commands and sends appropriately timed messages to UAV

### Messages
* Gesture Message
header
uint16 gesture_number "here's what I understood from the programmer"

* Modular Command Message
header
string command "heres what I want to do"

* Queue Message
header
float64[n][5] uav_commands "do this set of things, UAV"

### Services


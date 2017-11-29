 /*
 * Author: Dennis Melamed
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <string.h>
#include <queue>
#include "crazyFrog/MacroRequest.h"
#include <sstream>
#include <fstream>
#include <string>

#define POSITION_ERROR .1
enum Direction{x, y, z, w};

struct Command
{
	Direction a;
	int amount;
}current;

struct Position
{
	double x;
	double y;
	double z;
} start_position;

geometry_msgs::PoseStamped msg;
std::queue <Command> command_queue;
int current_program_number = 0;

void addMacroCommandsToQueue(int macro_number)
{
	char macro_num = (char) macro_number;
	std::string macro_n("../macros/macro");
        macro_n	+= &macro_num;
	macro_n +=".csv";
	
	std::ifstream macro_file(macro_n.c_str());
	std::string line;
	while (std::getline(macro_file, line))
	{
		std::istringstream iss(line);
		char a;
		int b;
		if (!(iss >> a >> b)) { break; } // error
		Command new_command;
		new_command.a = static_cast<Direction>(a-120);
		new_command.amount = b;
		command_queue.push(new_command);
	}
}

void programCallback(const crazyFrog::MacroRequest& request)
{
	if(request.program_counter > current_program_number)
	{
		current_program_number +=1;
		addMacroCommandsToQueue(request.macro_number);
	}
}

void updateTransform(tf::StampedTransform& transform, tf::TransformListener& listener)
{
	try
	{
		listener.lookupTransform("/cam_pos", "/world", ros::Time(0), transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
}


void init(tf::StampedTransform& transform)
{	
	start_position.x = transform.getOrigin().x();
	start_position.y = transform.getOrigin().y();
	start_position.z = transform.getOrigin().z();

	ROS_INFO("current transform x: %f y %f z %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());

}

bool posWithinError(double start_pos, double current_pos, double goal_pos, double allowable_error)
{
	double ideal_difference = goal_pos-start_pos;
	double current_difference = current_pos-start_pos;
	double position_error = ideal_difference-current_difference;
	if(-allowable_error < position_error && position_error < allowable_error)
	{
		return true;
	}
	else
	{
		return false;
	}
}
bool goalAchieved(tf::StampedTransform& transform)
{
	ROS_INFO("current transform x: %f y %f z %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());

   	if(  posWithinError(start_position.x, transform.getOrigin().x(), msg.pose.position.x, POSITION_ERROR) &&
   	     posWithinError(start_position.y, transform.getOrigin().y(), msg.pose.position.y, POSITION_ERROR) &&
   	     posWithinError(start_position.z, transform.getOrigin().z(), msg.pose.position.z, POSITION_ERROR))
   	{
   	     return true;
   	}
   	return false;

}

void updateMessage(tf::StampedTransform& transform)
{
	
      if(!command_queue.empty() && goalAchieved(transform))
      { 
		current = command_queue.front();
		command_queue.pop();
	  	msg.pose.position.x = (current.a == x) ? (msg.pose.position.x + current.amount) : (msg.pose.position.x);	
	  	msg.pose.position.y = (current.a == y) ? (msg.pose.position.y + current.amount) : (msg.pose.position.y);	
	  	msg.pose.position.z = (current.a == z) ? (msg.pose.position.z + current.amount) : (msg.pose.position.z);	
	  	// handle (current.a == w) // a wait command
		init(transform);
      }
}

int main(int argc, char **argv)
{
	//Initializes ROS, and sets up a node
	ros::init(argc, argv, "runtime");
	ros::NodeHandle nh;


  	ros::Publisher  pub  =nh.advertise<geometry_msgs::PoseStamped>("/goal", 100);
 	ros::Subscriber sub  =nh.subscribe("/crazyFrog/currentProgram", 100, programCallback);

   	tf::StampedTransform transform;
   	tf::TransformListener listener;


   	updateTransform(transform, listener);
   	init(transform);
   	ros::AsyncSpinner spinner(4); // Use 4 threads
   	spinner.start();

   	ros::Rate rate(10);
   	

   	while(ros::ok())
   	{
   		updateTransform(transform, listener);
   	    updateMessage(transform);
   	   	pub.publish(msg);
   		rate.sleep();
   	}
   	return 0;
}

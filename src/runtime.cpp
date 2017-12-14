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
#include <ros/package.h>

#define POSITION_ERROR .1
enum Direction{w, x, y, z};

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
std_msgs::String current_msg;
std_msgs::String queue_msg;
std::queue <Command> command_queue;
int current_program_number = 0;
int wait_timer = 0;

void addMacroCommandsToQueue(int macro_number)
{

	std::string package("crazy_frog");
	std::stringstream file_path;
	file_path << ros::package::getPath(package) << "/macros/macro" << macro_number << ".csv";
	
	ROS_INFO("adding from file: %s" , file_path.str().c_str());

	std::ifstream macro_file(file_path.str().c_str());
	std::string line;
	while (std::getline(macro_file, line))
	{
		std::istringstream iss(line);
		char a;
		int b;
		if (!(iss >> a >> b)) { break; } // error
		ROS_INFO("command added: %c, distance: %d", a, b);
		Command new_command;
		new_command.a = static_cast<Direction>(a-119);
		ROS_INFO("added command with direction: %d", new_command.a);
		new_command.amount = b;
		command_queue.push(new_command);
		std::stringstream queue_string;
		queue_string << queue_msg.data << "\n" << a << " " << b;
		queue_msg.data = queue_string.str().c_str();
		
	}
	
}

void programCallback(const crazyFrog::MacroRequest& request)
{
	if(request.program_counter > current_program_number)
	{
		current_program_number +=1;
		addMacroCommandsToQueue(request.macro_number);
		ROS_INFO("adding a new program to queue");
	}
	ROS_INFO("currently running program # %d, which is macro # %d", current_program_number, request.macro_number);
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
	
//	ROS_INFO("current transform x: %f y %f z %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
}


void init(tf::StampedTransform& transform)
{	
	start_position.x = transform.getOrigin().x();
	start_position.y = transform.getOrigin().y();
	start_position.z = transform.getOrigin().z();
//ROS_INFO("current transform x: %f y %f z %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());

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
	ROS_INFO("wait_timer: %d ", wait_timer);
   	if(  posWithinError(start_position.x, transform.getOrigin().x(), msg.pose.position.x, POSITION_ERROR) &&
   	     posWithinError(start_position.y, transform.getOrigin().y(), msg.pose.position.y, POSITION_ERROR) &&
   	     posWithinError(start_position.z, transform.getOrigin().z(), msg.pose.position.z, POSITION_ERROR) &&
	     wait_timer ==0)
   	{
   	     return true;
   	}
	else if (wait_timer > 0)
	{
		wait_timer -= 1;
	}
   	return false;

}

void updateMessage(tf::StampedTransform& transform)
{
	
      if(!command_queue.empty() && goalAchieved(transform))
      { 
		ROS_INFO("setting new goal pos");
		current = command_queue.front();
		std::stringstream current_string;
		current_string << (char) (current.a+119) << " " << current.amount;
		current_msg.data = current_string.str().c_str();
		std::string queue_string(queue_msg.data);
		queue_string.erase(0, queue_string.find("\n")+1);
		queue_msg.data = queue_string.c_str();
		command_queue.pop();
		ROS_INFO("setting new goal position with direction: %d, at point: %f", current.a, msg.pose.position.x+current.amount);
		Direction a = x;
		Direction b = y;
		Direction c = z;
		Direction d = w;
	  	msg.pose.position.x = (current.a == a) ? (msg.pose.position.x + current.amount) : (msg.pose.position.x);	
	  	msg.pose.position.y = (current.a == b) ? (msg.pose.position.y + current.amount) : (msg.pose.position.y);	
	  	msg.pose.position.z = (current.a == c) ? (msg.pose.position.z + current.amount) : (msg.pose.position.z);	
		if(current.a == d)
		{
			wait_timer = current.amount*10;
		}
		ROS_INFO("actual goal pose set: %f, %f, %f", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
	  	// handle (current.a == w) // a wait command
		init(transform);
      }
      if(command_queue.empty() && goalAchieved(transform))
      {
		current_msg.data = "";
		queue_msg.data = "";
      }
}

int main(int argc, char **argv)
{
	//Initializes ROS, and sets up a node
	ros::init(argc, argv, "runtime");
	ros::NodeHandle nh;


  	ros::Publisher  pub  =nh.advertise<geometry_msgs::PoseStamped>("/goal", 100);
	ros::Publisher  queue_pub = nh.advertise<std_msgs::String>("/crazyFrog/command_queue", 100);
	ros::Publisher  current_pub = nh.advertise<std_msgs::String>("/crazyFrog/current_command",100);
 	ros::Subscriber sub  =nh.subscribe("/crazyFrog/current_program", 100, programCallback);

   	tf::StampedTransform transform;
   	tf::TransformListener listener;


   	updateTransform(transform, listener);
   	init(transform);
   	ros::AsyncSpinner spinner(4); // Use 4 threads
   	spinner.start();

   	ros::Rate rate(10);
   	
	msg.pose.position.x = start_position.x;
	msg.pose.position.y = start_position.y;
	msg.pose.position.z = start_position.z;
	msg.header.frame_id = "cam_pos";
   	while(ros::ok())
   	{
   		updateTransform(transform, listener);
   		updateMessage(transform);
   	   	pub.publish(msg);
		queue_pub.publish(queue_msg);
		current_pub.publish(current_msg);
   		rate.sleep();
   	}
   	return 0;
}

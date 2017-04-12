/*
	Thao Nguyen
	Long Nguyen 
	Sharayu Shetty 
	Sanika Deshpande 
	Rocio Salguero 
	
	CPSC 481 - Assignment 2
	Description: This program makes the turtle autonomously navigate
				 on a rectangle loop. The turtle can start on any
				 location and follow a rectangular path. Once it 
				 comes back to the original location, it stops.
*/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

using namespace std;

// Create a publisher object
ros::Publisher velocity_publisher;

const double PI = 3.14159265359;

// moves with a certain linear velocity for a certain distance
void move(double speed, double distance)
{
	// Create the message
	geometry_msgs::Twist vel_msg;

	// Initialize the message
	vel_msg.linear.x = speed; 
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

	// Create and set initialize time and distance
	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;

	ros::Rate rate(100);

	do
    {
		// Publish the message	
		velocity_publisher.publish(vel_msg);
		// Create and set new time
		double t1 = ros::Time::now().toSec();
		// Calculate the current distance
		current_distance = speed * (t1-t0);
		// Return control back to ros
		ros::spinOnce();
		// Delay the program
		rate.sleep();
		
	} while(current_distance < distance);

	vel_msg.linear.x = 0;	// Reset the linear velocity
	// Publish the message
	velocity_publisher.publish(vel_msg);
}

// makes the turtle turn with a specified angular velocity
void rotate (double angular_speed, double relative_angle)
{
	// Create the message
	geometry_msgs::Twist vel_msg;
	
	// Initialize the message
	vel_msg.linear.x =0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;

	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = angular_speed;
	
	// Create and initialize the initial time and angle
	double t0 = ros::Time::now().toSec();
	double current_angle = 0.0;

	ros::Rate rate(100);

	do
	{
		// Publish the message
		velocity_publisher.publish(vel_msg);
		// Create and set new time
		double t1 = ros::Time::now().toSec();
		// Calculate the current angle
		current_angle = angular_speed * (t1-t0);
		// Return control to ros
		ros::spinOnce();
		// Delay the program
		rate.sleep();
		
	} while (current_angle < relative_angle);

	vel_msg.angular.z = 0;	// Reset angular velocity
	// Publish the message
	velocity_publisher.publish(vel_msg);
}

// converts angles from degree to radians
double degreesToRadians(double angle_in_degrees)
{
	return (angle_in_degrees *PI /180.0);
}

int main(int argc, char **argv)
{
	// Initialize the ROS system
	ros::init(argc, argv, "boxturtle");
	// Become a node n
	ros::NodeHandle n;
	
	// Initialize velocity_publisher
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);      

	//Set rate at 2 Hz until the node shuts down 
	ros::Rate rate(2);

	move(1,5);	// moves to the right
	rate.sleep();	

	rotate(degreesToRadians(10), degreesToRadians(90));	// rotate 90 degrees
	rate.sleep();
	
	move(1,2);	// moves upwards
	rate.sleep();

	rotate(degreesToRadians(10), degreesToRadians(90));	// rotates 90 degrees
	rate.sleep();

	move(1,5);	// moves to the left
	rate.sleep();

	rotate(degreesToRadians(10), degreesToRadians(90));	// rotates 90 degrees
	rate.sleep();

	move(1,2);	// moves downwards
	rate.sleep();

	return 0;
}

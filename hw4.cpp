/*
	Rocio Salguero 	
	Thao Nguyen
	Long Nguyen 
	Sharayu Shetty 
	Sanika Deshpande 
	
	CPSC 481 - Assignment 4
	compile: catkin_make 
	source devel/setup.bash	
	run: rosrun turtlesim turtlesim_node
	     rosrun hw4 hw4test
	     rosrun hw4 hw4	
		
	Description: mainTurtle moves to capture target turtles (Ti) 
	and avoid collision and villain turtles (Xi)

	Last updated: 05/20/17 03:40PM
*/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <string>
#include <stdlib.h>  
#include <time.h>  
#include <math.h> 
#include <ctime>

#define vSIZE 3		//max number of villain turtles
#define tSIZE 3		//max number of target turtles

using namespace std;

// Create a publisher object
ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose mainturtle;

ros::Subscriber X1;
ros::Subscriber X2;
ros::Subscriber X3;
ros::Subscriber T1;
ros::Subscriber T2;
ros::Subscriber T3;

const double PI = 3.14159265359;  
double turtleDist = 0.0;  // to calculate distance from start point for g()
double MAX_VAL = 100.0;	  
double distanceTraveled = 0.0; // to calculate distance traveled using vel * time

//struct to keep values of villain/target turtles 
struct turtles {
	double x; 
	double y;
	double theta;  
	string turtlename; 
	bool caught; 
};

//struct to define a coordinate with x- and y- values
struct coord {
	double nextx; 
	double nexty; 
};

//Arrays to keep track of villain/target turtles information
turtles villains[vSIZE]; 
turtles targets[tSIZE];

//struct to store info learned for machine learning
struct knowledge {
	bool rotate;
	double lastx;
	double lasty; 
	double speedx; 
	double speedy; 
	time_t lasttime; 	
};

// array of training data set
knowledge tinfo[tSIZE];


/*-------------Functions Declaration---------------*/

//used for subscriber to get the pose of mainTurtle
void getLocation(const turtlesim::Pose::ConstPtr & pose_message); 
double distFunc(double x1, double y1, double x2, double y2);

void x1Loc(const turtlesim::Pose::ConstPtr & pose_message); 
void x2Loc(const turtlesim::Pose::ConstPtr & pose_message); 
void x3Loc(const turtlesim::Pose::ConstPtr & pose_message); 
void t1Loc(const turtlesim::Pose::ConstPtr & pose_message); 
void t2Loc(const turtlesim::Pose::ConstPtr & pose_message); 
void t3Loc(const turtlesim::Pose::ConstPtr & pose_message); 

//functions to check if mainTurtle is near a villain/target 
bool check_fail_point(double x, double y);

// moves with a certain linear velocity for a certain distance
void move(double speed, double distance);
//rotates to the point relative to where mainturtle is at 
void relative_rotate(double x_val, double y_val); 
// makes the turtle turn with a specified angular velocity
void rotate (double angular_speed, double angle, bool clockwise);
// converts angles from degree to radians
double degreesToRadians(double angle_in_degrees);

//functions for our heurstic functions
double f_func(double nextX, double nextY, int turtleInx);
double g_func();
double h_func(double nextX, double nextY, int turtle);

//functions to make turtle move
void moveAI();
int closest_target();
int check_target();  
void killtargetturtle(int index); 
coord evaluate(int currtarget);

/*-------------End Functions Declaration---------------*/

void initial() {
	for (int i= 0; i <tSIZE; i++) {
		tinfo[i].lasty = tinfo[i].lastx = -1; 
		tinfo[i].lasttime = 0; 
	}
}
/*-------------Start Main Function---------------*/
int main(int argc, char **argv)
{
	// Initialize the ROS system
	ros::init(argc, argv, "hw4");

	// Become a node n
	ros::NodeHandle n;
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000); 	
	pose_subscriber = n.subscribe("/turtle1/pose", 1000, getLocation);     

	X1 = n.subscribe("/X1/pose", 1000, x1Loc); 
	X2 = n.subscribe("/X2/pose", 1000, x2Loc); 
	X3 = n.subscribe("/X3/pose", 1000, x3Loc);
 
	T1 = n.subscribe("/T1/pose", 1000, t1Loc); 
	T2 = n.subscribe("/T2/pose", 1000, t2Loc); 
	T3 = n.subscribe("/T3/pose", 1000, t3Loc); 
	initial(); 
	move(0.5, 0.1);	//initial move so all other turtles start moving
	moveAI();
	// Display total distance traveled
	ROS_INFO("Traveled calculated distance:%.5f Actual distance:%.5f", distanceTraveled, turtleDist); 

	return 0;
} 
/*-------------End Main Function---------------*/

/*-------------Movement functions--------------*/

//We need to make a function what will keep moving the turtle
//till it catches all target turtles and uses the heuristic functions
//f(x) to find the hearistic value of next move - use f_func(x,y)
//must test if failed - use check_fail(x,y) and system("rosservice call /kill mainTurtle") 
//If captured a target turtle - use check_target() and killtargetturtle() 
//To move to location - use move(speed, distance) and relative_rotate(x,y) 

double moveDist = 0.1;
 
int closest_target() {
	int closest = -1, i;
	double dist = 999; 	
	for(i=0; i < tSIZE; i++) {
		if(distFunc(mainturtle.x, mainturtle.y, targets[i].x, targets[i].y) < dist 
				&& targets[i].caught == false) {
			dist = distFunc(mainturtle.x, mainturtle.y, targets[i].x, targets[i].y); 
			closest = i; 
		}
	}
	return closest; 
}

void moveAI() {
	int turtle; 
	coord nextMove; 
	int inx; 	
	
	turtle = closest_target(); 

	while(true) 
	{
		turtle = closest_target(); 
		if(turtle < 0 || turtle >= 3) {ROS_INFO("Out of bounds!"); return; }

		//evaluate min of near by coord 
		nextMove = evaluate(turtle); 
		
		//Move to that location
		relative_rotate(nextMove.nextx, nextMove.nexty); 
		
		move(2,moveDist);
		
		// Update distance traveled
		distanceTraveled += 0.5 * (moveDist/0.5); // distance += speed * time

		//ROS_INFO("Turtle x:%.5f y:%.5f diff x: %.5f y:%.5f", mainturtle.x, mainturtle.y, 
			//mainturtle.x-nextMove.nextx, mainturtle.y - nextMove.nexty);
	
		// check if there is any T turtle nearby to capture
		inx = check_target();		
		if(inx != -999) 
		{
			killtargetturtle(inx); 			
		}	
				
		//check if target captured 
		if(targets[0].caught && targets[1].caught && targets[2].caught) break; 		
	}
	
	ROS_INFO("Captured all turtles! You win!");
}

coord evaluate(int currtarget) {
	double theta=0; 
	
	coord min; 
	double mx, my; 
	double current;
	double minVal = 999; 

	for(theta = 0; theta <= 2*PI; theta+=degreesToRadians(2) ) {
		mx = (moveDist*cos(theta) ) + mainturtle.x; 
		my = (moveDist*sin(theta) ) + mainturtle.y; 
		current = f_func(mx,my,currtarget);

		if(current < minVal) {
			minVal = current;
			min.nextx = mx; 
			min.nexty = my;  
		}
	}
	//ROS_INFO("DONE min val :%.5f x:%.5f y:%.5f",minVal, min.nextx, min.nexty); 
	
	return min; 
}

/*-------------End Movement Functions--------------*/ 

/*-------------Heuristic Functions--------------*/
//caltulate the heuristic function of the next location 
double f_func(double nextX, double nextY, int turtleInx) {
	return g_func() + h_func(nextX, nextY, turtleInx); 	
} 
double g_func() {
	return turtleDist;
} 
double h_func(double nextX, double nextY, int turtle){
	if( check_fail_point(nextX,nextY) == true ){
		return MAX_VAL; 
	}

	if(turtle == 0 || turtle == 1) {
		//use generalization for these two turtles 

		//check if moved 
		if(tinfo[turtle].lastx == -1 && tinfo[turtle].lasty == -1) {
			//go to where it is 
			return distFunc(nextX, nextY, targets[turtle].x, targets[turtle].y); 	
		}
		else if (distFunc(tinfo[turtle].lastx,0, targets[turtle].x, 0) <= .5 && 
				distFunc(0,tinfo[turtle].lasty,0,targets[turtle].y) <= .5) {
			//didnt move go to where it is
			return distFunc(nextX, nextY, targets[turtle].x, targets[turtle].y); 	
		}

		time_t curr_time = time(NULL);
		//find the speed 
		tinfo[turtle].speedx = (targets[turtle].x - tinfo[turtle].lastx)/(curr_time - tinfo[turtle].lasttime); 
		tinfo[turtle].speedy = (targets[turtle].y - tinfo[turtle].lasty)/(curr_time - tinfo[turtle].lasttime); 

		//generalize speed almost 0 ergo ignore that direction
		double dist; double next; 
		if(tinfo[turtle].speedx <= 0.1 && tinfo[turtle].speedx >=0.1) 
		{
			dist = tinfo[turtle].speedx * (curr_time - tinfo[turtle].lasttime); 
			
			//predict next y value 
			next = targets[turtle].y + dist; 

			//update info 
			tinfo[turtle].lastx = targets[turtle].x;
			tinfo[turtle].lasty = targets[turtle].y;
			tinfo[turtle].lasttime = time(NULL);

			return distFunc(nextX, nextY, targets[turtle].x, next); 
		} 
		else if(tinfo[turtle].speedy <= 0.1 && tinfo[turtle].speedy >=0.1) 
		{
			dist = tinfo[turtle].speedy * (curr_time - tinfo[turtle].lasttime); 
			//predict next x value 
			next = targets[turtle].x + dist; 

			//update info 
			tinfo[turtle].lastx = targets[turtle].x;
			tinfo[turtle].lasty = targets[turtle].y;
			tinfo[turtle].lasttime = time(NULL);

			return distFunc(nextX, nextY, next, targets[turtle].y); 
		}
		
		
		
	}
	else //turtle T3
	{
		//have to keep track of turning, default heuristic for now
		return distFunc(nextX, nextY, targets[turtle].x, targets[turtle].y);
	}	
	

	//return distFunc(nextX, nextY, targets[turtle].x, targets[turtle].y); 		
	
} 

/*-------------End Heuristic Functions--------------*/


/*---------Functions for Location Values------------*/
void getLocation(const turtlesim::Pose::ConstPtr & pose_message) {
	mainturtle.x = pose_message->x;
	mainturtle.y = pose_message->y;
	mainturtle.theta = pose_message->theta;
  //ROS_INFO("info x y: %.2f %02f", mainturtle.x, mainturtle.y);
}

double distFunc(double x1, double y1, double x2, double y2) {
	return sqrt( (x2-x1)*(x2-x1)+ (y2-y1)*(y2-y1) );
}
void x1Loc(const turtlesim::Pose::ConstPtr & pose_message) {
	villains[0].x = pose_message->x;
	villains[0].y = pose_message->y; 
	villains[0].theta = pose_message->theta; 
}
void x2Loc(const turtlesim::Pose::ConstPtr & pose_message) {
	villains[1].x = pose_message->x;
	villains[1].y = pose_message->y; 
	villains[1].theta = pose_message->theta; 
}
void x3Loc(const turtlesim::Pose::ConstPtr & pose_message) {
	villains[2].x = pose_message->x;
	villains[2].y = pose_message->y; 
	villains[2].theta = pose_message->theta; 
}
void t1Loc(const turtlesim::Pose::ConstPtr & pose_message) {
	targets[0].x = pose_message->x;
	targets[0].y = pose_message->y; 
	targets[0].theta = pose_message->theta; 
	//ROS_INFO("Turtle 1: (%f, %f)", targets[0].x, targets[0].y);
}
void t2Loc(const turtlesim::Pose::ConstPtr & pose_message) {
	targets[1].x = pose_message->x;
	targets[1].y = pose_message->y; 
	targets[1].theta = pose_message->theta; 
}
void t3Loc(const turtlesim::Pose::ConstPtr & pose_message) {
	targets[2].x = pose_message->x;
	targets[2].y = pose_message->y; 
	targets[2].theta = pose_message->theta; 
}
/*---------End Location Functions------------*/ 

//returns true if out of bounds or near villain turtle 
//0.03 distance error 
bool check_fail_point (double x, double y) {
	if(x < 0.0 || 11.0 < x 
		|| y < 0.0 || 11.0 < y) 
		return true; 
	
	double vx, vy; 
	double dist_err = 0.53, dist_check;
	for(int j=0; j < vSIZE; j++) {
		vx = villains[j].x;
		vy = villains[j].y;
		
		dist_check = distFunc(x, y, vx, vy);
		
		//check within range 
		if(dist_check <= dist_err) 
			return true; 
		
 	}

	return false; 
}
//returns the target[i] index when mainTurtle is near, else return -999 (not near any target turtle)
int check_target() {
	double vx, vy; 
	double dist_err = 0.5, dist_check;
	for(int i=0; i < vSIZE; i++) {
		
		vx = targets[i].x;
		vy = targets[i].y;
		//ROS_INFO ("Turtle %d is currently at (%f,%f)", i+1, vx, vy);
		dist_check = distFunc(mainturtle.x, mainturtle.y, vx, vy);
		//ROS_INFO("Distance is %f", dist_check);
		//check within range 
		if(dist_check <= dist_err) 
			return i;
		
 	}
	return -999; 
}
/*-------------End Check Functions------------------*/

//kills the target turtle mainTurtle went close to
void killtargetturtle(int index) {
	if(targets[index].caught == true) return;
	
	char buffer[30];  
	snprintf(buffer, sizeof(buffer), "rosservice call /kill T%d", (index+1)); 
	targets[index].caught = true; 
	
	
	
	ROS_INFO("Captured turtle:%s", buffer);
 
	system(buffer);
}

/*-------------End Check Functions------------------*/

/*-------------Basic move functions ------------------*/
// moves with a certain linear velocity for a certain distance
void move(double speed, double distance) {
	 
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

	ros::Rate rate(200);
	int inx;
	

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

	turtleDist += current_distance;

	vel_msg.linear.x = 0;	// Reset the linear velocity
	// Publish the message
	velocity_publisher.publish(vel_msg);
}

void relative_rotate(double x_val, double y_val) {
	bool clockwise = true; 
	double thetaPrime = 0, thetaRel = 0; 
	double err = 0.005; 
	double m = 0; 

	//find the slope and arctan 
	double y_slope = (y_val - mainturtle.y);
	double x_slope = (x_val - mainturtle.x); 
 
	//special case if x_slope == 0 within .5% range of 0 
	if(err > x_slope && x_slope > -err) {
		if(y_slope < 0) //if negative 
			thetaPrime = -PI/2; //-90 deg
		else //y is positive 
			thetaPrime = PI/2;  //90 deg 
	}
	else { 
		m = y_slope / x_slope;
		thetaPrime = atan(m); 
	}
	
	//make the angle relative to the quardrant 
	//get the positive angle 
	//Quadrant 1
	if( x_val >= mainturtle.x && y_val >= mainturtle.y) 
		; 
	//Quadrant 2 thetaPrime was negative
	else if( x_val < mainturtle.x && y_val > mainturtle.y) 
		thetaPrime = PI + thetaPrime; 
	//Quadrant 3 
	else if ( x_val < mainturtle.x && y_val < mainturtle.y) 
		thetaPrime = PI + thetaPrime; 
	//Quadrant 4 thetaPrime was negative
	else if( x_val >= mainturtle.x && y_val <= mainturtle.y) 
		thetaPrime = 2*PI + thetaPrime; 
		

	//ROS_INFO("Tan:%.5f", thetaPrime);
	if(thetaPrime > mainturtle.theta) {
		thetaRel = thetaPrime - mainturtle.theta;
		clockwise = false; //go counter clockwise
	}
	else { //if theta > thetaPrime 
		thetaRel = mainturtle.theta - thetaPrime;
		clockwise = true; //go counter clockwise
	}
	//ROS_INFO("theta:%.5f thetaprime:%.5f clockwise:%d ", thetaRel * 180 / PI, thetaPrime * 180 / PI, clockwise ); 
	rotate (thetaRel , thetaRel, clockwise);

}

void rotate (double angular_speed, double relative_angle, bool clockwise) {

	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	vel_msg.linear.x =0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

	if (clockwise)
		vel_msg.angular.z =-abs(angular_speed);
	else
		vel_msg.angular.z =abs(angular_speed);

	double current_angle = 0.0;
	double t0 = ros::Time::now().toSec();
	ros::Rate loop_rate(200);
	do{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	} while(current_angle<relative_angle);

	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);

}

// converts angles from degree to radians
double degreesToRadians(double angle_in_degrees)
{
	return (angle_in_degrees *PI /180.0);
}

/*------------End basic move functions---------------*/

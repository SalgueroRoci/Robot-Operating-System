/*
	Rocio Salguero 	
	Thao Nguyen
	Long Nguyen 
	Sharayu Shetty 
	Sanika Deshpande 
	
	
	CPSC 481 - Assignment 3
	compile: catkin_make 
	source devel/setup.bash	
	launch: roslaunch turtlecatch boxturtle.launch
		
	Description: mainTurtle moves to capture target turtles (Ti) 
	and avoid collision and villain turtles (Xi)
*/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <turtlesim/Spawn.h>
#include <string>
#include <stdlib.h>  
#include <time.h>  
#include <math.h> 

#define vSIZE 4
#define tSIZE 3

using namespace std;

// Create a publisher object
ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
//ros::Subscriber villian_subscriber;
//turtlesim::Pose villianturtle; 
turtlesim::Pose mainturtle;

const double PI = 3.14159265359;  
double turtleDist = 0;  //to calculate distance traveled
double MAX_VAL = 100;

//struct to keep values of villain/target turtles 
struct turtles {
	double x; 
	double y;
	double theta;  
	string turtlename; 
	bool caught; 
};

struct coord {
	double nextx; 
	double nexty; 
};

//Arrays to keep track of villain/target turtles information
turtles villains[vSIZE]; 
turtles targets[tSIZE];
//Used to spawn the turtles 
turtlesim::Spawn srv;


/*-------------Functions---------------*/

//used for subscriber to get the pose of mainTurtle
void getLocation(const turtlesim::Pose::ConstPtr & pose_message); 
double distFunc(double x1, double y1, double x2, double y2);

//spawn villain/target turtles away from turtle 1 and within boundry 
void create_enemyturtles(ros::ServiceClient addt); 
void create_targetturtles(ros::ServiceClient addt); 
void create_mainturtle(ros::ServiceClient addt); 
double createRand(); 
bool checkduplicate(double num,  int XorY);

//functions to check if mainTurtle is near a villain/target 
bool check_fail(); 
bool check_fail_point(double x, double y);
int check_target();  
void killtargetturtle(int index); 

// moves with a certain linear velocity for a certain distance
void move(double speed, double distance);
//rotates to the point relative to where mainturtle is at 
void relative_rotate(double x_val, double y_val); 
// makes the turtle turn with a specified angular velocity
void rotate (double angular_speed, double angle, bool clockwise);
// converts angles from degree to radians
double degreesToRadians(double angle_in_degrees);

//functions for our heurstic function and to make turtle move
double f_func(double nextX, double nextY, int turtleInx);
double g_func();
double h_func(double nextX, double nextY, int turtle);

void moveAI();
coord evalute(int currtarget);

/*-------------End--Functions---------------*/


/*-------------Start Main---------------*/
int main(int argc, char **argv)
{
	// Initialize the ROS system
	ros::init(argc, argv, "turtlecatch");

	// Become a node n
	ros::NodeHandle n;
	ros::ServiceClient add_turtle = n.serviceClient<turtlesim::Spawn>("spawn");
	
	//Create turtles 
	create_mainturtle(add_turtle); 
	create_targetturtles(add_turtle); 
	create_enemyturtles(add_turtle); 

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/mainTurtle/cmd_vel", 1000); 
	pose_subscriber = n.subscribe("/mainTurtle/pose", 1000, getLocation);     
	
	moveAI();

	return 0;
} 
/*-------------End--Main---------------*/

//---------movement function-------------

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

	while(true) {
		if(turtle < 0 || turtle >= 3) {ROS_INFO("Out of bounds!"); return; }

		//evalute min of near by coord 
		nextMove = evalute(turtle); 

		//Move to that location
		relative_rotate(nextMove.nextx, nextMove.nexty); 
		move(0.5,moveDist); 

		//ROS_INFO("Turtle x:%.5f y:%.5f diff x: %.5f y:%.5f", mainturtle.x, mainturtle.y, 
			//mainturtle.x-nextMove.nextx, mainturtle.y - nextMove.nexty);

		//check if target captured 
		inx = check_target();		
		if(inx != -1) {
			killtargetturtle(inx); 
			//check if all were caught
			if(targets[0].caught && targets[1].caught && targets[2].caught) break; 
			turtle = closest_target(); 
		}
		//check if failed 
		else if(check_fail() ) {
			ROS_INFO("Main turtle killed. Failed!");
			system("rosservice call /kill mainTurtle");
			return; 					
		}		
		
	}
	
	ROS_INFO("Captured all turtles you win!");

}
coord evalute(int currtarget) {
	double theta=0; 
	
	coord min; 
	double mx, my; 
	double current;
	double minVal = 999; 

	for(theta = 0; theta <= 360; theta+=10 ) {
		mx = (moveDist*cos(degreesToRadians(theta) ) ) + mainturtle.x; 
		my = (moveDist*sin(degreesToRadians(theta) ) ) + mainturtle.y; 
		current = f_func(mx,my,currtarget);

		if(current < minVal) {
			minVal = current;
			min.nextx = mx; 
			min.nexty = my;  
		}
	}
	//ROS_INFO("min val :%.5f x:%.5f y:%.5f",minVal, min.nextx, min.nexty); 
	
	return min; 


}

//End movement 

//-----Heuristic functions-----
//caltulate the heuristic function of the next location 
double f_func(double nextX, double nextY, int turtleInx) {
	return g_func() + h_func(nextX, nextY, turtleInx); 
} 
double g_func() {
	return turtleDist;
} 
double h_func(double nextX, double nextY, int turtle){
	
	if( check_fail_point(nextX,nextY) == true || check_fail() ){
		return MAX_VAL; 
	}
	return distFunc(nextX, nextY, targets[turtle].x, targets[turtle].y); 		
	
} 

//Heauristics 


//---------Functions for location values----------------------
void getLocation(const turtlesim::Pose::ConstPtr & pose_message) {
	mainturtle.x = pose_message->x;
	mainturtle.y = pose_message->y;
	mainturtle.theta = pose_message->theta;
  //ROS_INFO("info x y: %.2f %02f", mainturtle.x, mainturtle.y);
}

double distFunc(double x1, double y1, double x2, double y2) {
	return sqrt( (x2-x1)*(x2-x1)+ (y2-y1)*(y2-y1) );
}
//end location 



//-------------check if near mainTurtle--------------------
bool check_fail() {
	
	if(mainturtle.x < 0.0|| 11.0 < mainturtle.x 
		|| mainturtle.y < 0.0 || 11.0 < mainturtle.y) 
		return true; 

	double vx, vy, perx, pery;
	for(int j=0; j < vSIZE; j++) {
		vx = villains[j].x;
		vy = villains[j].y;
		perx = vx*.1; 
		pery = vy*.1;

		//check within range 
		if((vx - perx) < mainturtle.x &&  mainturtle.x < (vx + perx) && (vy - pery) < mainturtle.y && mainturtle.y < (vy + pery)	) 
			return true; 
		
 	}

	return false; 
}
//returns true if out of bounds or near villain turtle 
//uses a 0.2 error 
bool check_fail_point(double x, double y) {
	double error = 0.2;
	
	if(x - error < 0.0 || x + error < 0.0 || x - error > 11.0 || x + error > 11.0 ) 
		return true; 
	if(y - error < 0.0 || y + error < 0.0 || y - error > 11.0 || y + error > 11.0 ) 
		return true; 
		
		
	double vx, vy, perx, pery;
	for(int j=0; j < vSIZE; j++) {
		vx = villains[j].x;
		vy = villains[j].y;
		perx = vx*.1; 
		pery = vy*.1;

		//check within range 
		if((vx - perx) < x - error &&  x - error < (vx + perx)  && (vy - pery) < y + error && y + error < (vy + pery) ) 
			return true; 
		else if((vx - perx) < x + error &&  x + error < (vx + perx)  && (vy - pery) < y + error && y + error < (vy + pery) ) 
			return true; 
		else if((vx - perx) < x + error &&  x + error < (vx + perx)  && (vy - pery) < y - error && y - error < (vy + pery) ) 
			return true; 
		else if((vx - perx) < x - error &&  x - error < (vx + perx)  && (vy - pery) < y - error && y - error < (vy + pery) ) 
			return true; 
 	}

	return false; 
}

//returns the target[i] index when mainTurtle is near, else return -1(not near any target turtle)
int check_target() {
	double vx, vy, perx, pery; 
	for(int j=0; j < tSIZE; j++) {
		vx = targets[j].x;
		vy = targets[j].y;
		perx = vx*0.05; 
		pery = vy*0.05;
		
		//check within range 
		if((vx - perx) < mainturtle.x && mainturtle.x < (vx + perx) && 
			(vy - pery) < mainturtle.y && mainturtle.y < (vy + pery)) 
			return j; 
 	}

	return -1; 
}

//kills the target turtle mainTurtle went close to
void killtargetturtle(int index) {
	if(targets[index].caught == true) return;
	targets[index].caught = true; 
	char buffer[30];  

	snprintf(buffer, sizeof(buffer), "rosservice call /kill %s", (targets[index].turtlename).c_str()); 
	ROS_INFO("turtle killed:%s", buffer);
 
	system(buffer);
}
//End Check turtles 




//--------------Functions to create turtles------------------
void create_mainturtle(ros::ServiceClient addt) {
	system("rosservice call /kill turtle1");
	srv.request.x = 0;
	srv.request.y = 0;
	srv.request.theta = 0;
	srv.request.name = "mainTurtle";
	ros::service::waitForService("spawn");
 
	addt.call(srv);
	
}

void create_targetturtles(ros::ServiceClient addt) {
	double x, y; 
	double deg = 0;
	string nameT; 
	char buffer[20];
    
	for(int i=0; i < tSIZE; i++) {
		srand(time(NULL));
		x = createRand(); 
		//while there is a duplicate, keep making a random number 
		while(checkduplicate(x, 1) ) {
			x = createRand(); 
		}
		y = createRand(); 
		//while there is a duplicate, keep making a random number 
		while(checkduplicate(y, 0) ) {
			y = createRand(); 
		}

		//make turtle
  		snprintf(buffer, sizeof(buffer), "T%d", (i+1));
		nameT = string(buffer); 
		ROS_INFO("Turtle %d being created location %.5f x %.5f y name: %s\n", (i+1), x, y, nameT.c_str()); 
		srv.request.x = x;
		srv.request.y = y; 
		srv.request.theta = deg;
		srv.request.name = nameT;
		ros::service::waitForService("spawn");
	 
		addt.call(srv);

		//add information to array 
		targets[i].x = x; 
		targets[i].y = y; 
		targets[i].theta = deg; 
		targets[i].turtlename = nameT; 
		
	}
}

void create_enemyturtles(ros::ServiceClient addt) {
	double x, y; 
	double deg = 0;
	string nameT; 
	char buffer[20];
    
	for(int i=0; i < vSIZE; i++) {
		srand(time(NULL));
		x = createRand(); 
		//while there is a duplicate, keep making a random number 
		while(checkduplicate(x, 1) ) {
			x = createRand(); 
		}
		y = createRand(); 
		//while there is a duplicate, keep making a random number 
		while(checkduplicate(y, 0) ) {
			y = createRand(); 
		}

		//make turtle
  		snprintf(buffer, sizeof(buffer), "X%d", (i+1));
		nameT = string(buffer); 
		
		srv.request.x = x;
		srv.request.y = y; 
		srv.request.theta = deg;
		srv.request.name = nameT;
		ros::service::waitForService("spawn");
	 
		addt.call(srv);

		//add information to array 
		villains[i].x = x; 
		villains[i].y = y; 
		villains[i].theta = deg; 
		villains[i].turtlename = nameT; 
		ROS_INFO("Turtle %d being created location %.5f x %.5f y name: %s\n", (i+1), villains[i].x, villains[i].y, nameT.c_str()); 
		
	}
}

//returns true if there is a duplicate 
//int XorY to check if on x or y axis. x = 1;y = 0 
bool checkduplicate(double num,  int XorY) {
	//at least 20% away from other turtles 
	double vari = num * 0.2; 

	//check the xaxis 
	if(XorY == 1) {
		int j=0; 
		for(j =0; j < vSIZE; j++) {
			if((villains[j].x - vari) < num && num < (villains[j].x + vari) )
				return true; 		
		}
		for(j =0; j < tSIZE; j++) {
			if((targets[j].x - vari) < num && num < (targets[j].x + vari) )
				return true; 		
		}
	}
	else if(XorY == 0) {
		int j=0; 
		for(j =0; j < vSIZE; j++) {
			if((villains[j].y - vari) < num && num < (villains[j].y + vari) )
				return true; 		
		}
		for(j =0; j < tSIZE; j++) {
			if((targets[j].y - vari) < num && num < (targets[j].y + vari))
				return true; 		
		}
	}
	//no duplicate found 
	return false; 
}

double createRand() {
	rand();
	double i = 0, d = 0;
	i = rand() % 950 + 100; //Gives a number between 100 to 950;
	d = i / 100;	//Gives range 1.0 to 9.5
	return d; 
}
//end spawn turtles 



//-------------Basic move functions --------------------
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
	thetaPrime = atan2(x_val, y_val); //returns postive theta in radians 

	//make the angle relative to the quardrant 
	if( x_val < mainturtle.x && y_val > mainturtle.y) 
		thetaPrime = PI - thetaPrime; 
	else if ( x_val < mainturtle.x && y_val < mainturtle.y) 
		thetaPrime = PI + thetaPrime; 
	else if( x_val > mainturtle.x && y_val < mainturtle.y) 
		thetaPrime = 2*PI - thetaPrime; 
		

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

void rotate (double angular_speed, double relative_angle, bool clockwise){

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

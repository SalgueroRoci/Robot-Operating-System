// In order to properly test your program, do the followings:
// (1) open a turtlesim window
// (2) run this program on another terminal
// (3) run your program on another terminal 
//-----------------------------------------------------------
#include <sstream>
#include <ros/ros.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <ros/master.h>
#include <boost/algorithm/string.hpp>

using namespace std;

const int MAX_TTURTLES = 3;
const int MAX_XTURTLES = 3;
const double DANGER_TOLERANCE = 0.5;
const double LOWER_LIMIT = 0.0;
const double UPPER_LIMIT = 11.0;
const double PI = 3.14159265359;
const int LOOP_SPEED = 10;
const int ANGULAR_SPEED = 10;

//for proportional controller
const double Kp = 1.0;
const double Ki = 0.02;
const double v0 = 2.0;
const double alpha = 0.5;

///////////////////////////////
namespace HW {
  struct Turtle {
     string turtlename;
     string posetopic;
     string veltopic;
     turtlesim::Pose pose;
  };
  static Turtle turtle1;
  static Turtle tturtles[MAX_TTURTLES];
  static Turtle xturtles[MAX_XTURTLES];
  static ros::ServiceClient sClient;
  static ros::ServiceClient kClient;

  static bool isTurtle1Moved = false;
  static int turtle1MoveCnt = 0;
  static turtlesim::Pose turtle1Origin;

  static string getTurtlename(const string topicname);
  static bool topicExist(const string topicname);
  static bool turtleExist(const string turtlename);
  static turtlesim::Pose getNonOverlappingPoint(char tType);
  static void createTurtles(char tType, int cnt);
  static double getDistance(double x1, double y1, double x2, double y2);
  static bool isTooClose(double x1, double y1, double x2, double y2, double threshhold);
  static bool isOffBoundary(double x, double y);
  static void removeTurtle1();
  void moveDistance(const double speed, const double distance, const bool isForward, const HW::Turtle& t, const ros::Publisher& velpublisher);
  void rotate(const double angular_speed, const double angle, const bool clockwise, const ros::Publisher& velpublisher);
  double degrees2radians(const double angle_degrees);
  void setDesiredOrientation(const double desired_angle_radians, const HW::Turtle& t, const ros::Publisher& velpublisher);
  void moveGoal(const turtlesim::Pose goal_pose, const double distance_tolerance, const HW::Turtle& t, const ros::Publisher& velpublisher);
};

//////////////
namespace HW {

string getTurtlename(const string topicname) {
  vector<string> elems;
  char lc_delim[2];
  lc_delim[0] = '/';
  lc_delim[1] = '\0';

  boost::algorithm::split(elems, topicname, boost::algorithm::is_any_of(lc_delim));
  return elems[1];
}

bool topicExist(const string topicname) {
  int i;
  string tname;
  ros::master::V_TopicInfo alltopics;

  //get all topic names 
  ros::master::getTopics(alltopics);

  for (int i=0; i<alltopics.size(); i++) {
     tname = alltopics[i].name;
     if (tname.compare(topicname) == 0) {
        return true;
     };
  };
  return false;
}

bool turtleExist(const string turtlename) {
  int i;
  string tname;
  ros::master::V_TopicInfo alltopics;

  //get all topic names 
  ros::master::getTopics(alltopics);

  for (int i=0; i<alltopics.size(); i++) {
     tname = getTurtlename(alltopics[i].name);
     if (tname.compare(turtlename) == 0) {
        return true;
     };
  };
  return false;
}

turtlesim::Pose getNonOverlappingPoint(char tType) {
  turtlesim::Pose xp;
  bool tooclose = false;
  int i;
  int ocnt=0;

  xp.x = double((rand() % 10) + 1.0);
  xp.y = double((rand() % 10) + 1.0);

  while (true) {
    if (HW::isTooClose(HW::turtle1.pose.x, HW::turtle1.pose.y, xp.x, xp.y, DANGER_TOLERANCE)) 
        tooclose = true;
    else if (tType == 'T')
            break; //out of while loop
    else { //X turtle needs to check all T turtles
       for (i=0; i<MAX_TTURTLES; i++) {
           if (HW::isTooClose(HW::tturtles[i].pose.x, HW::tturtles[i].pose.y, xp.x, xp.y, DANGER_TOLERANCE)) {
              tooclose = true;
              break; //out of for loop and regenerate a point
           };
       };
    };

    if (!tooclose) //checking for X turtle case
       break; //out of while loop

    if (ocnt>1000) { //only to check abnormality
       ROS_INFO_STREAM("chk: " << xp.x << "," << xp.y << "\n");
       break; //possibly wrong so exit 
    };
    //generate another random pose
    xp.x = double((rand() % 10) + 1.0);
    xp.y = double((rand() % 10) + 1.0);
    tooclose = false;
    ocnt++;
    ROS_INFO_STREAM(".");
  };
  return xp;
}

void createTurtles(char tType, int cnt) {
  int i;
  stringstream tname, vname, cmdstr;
  bool success = false;
  turtlesim::Spawn::Request req;
  turtlesim::Spawn::Response resp;
  turtlesim::Pose nop;

  for (i=0; i<cnt; i++) {
     tname.clear();
     tname.str("");
     tname << tType << i + 1;
     req.name = tname.str();
     nop = HW::getNonOverlappingPoint(tType);
     req.x = nop.x;
     req.y = nop.y;
     req.theta = M_PI/2; //face up for target turtles

     tname.clear();
     tname.str("");
     tname << "/" << req.name << "/pose";
     vname.clear();
     vname.str("");
     vname << "/" << req.name << "/cmd_vel";

     //fill out turtles tables for pose tracking
     if (tType == 'X') {
        req.theta = 3.0*req.theta; //change to face down for villain turtles
        HW::xturtles[i].turtlename = req.name;
        HW::xturtles[i].posetopic = tname.str();
        HW::xturtles[i].veltopic = vname.str();
        HW::xturtles[i].pose.x = req.x;
        HW::xturtles[i].pose.y = req.y;
        HW::xturtles[i].pose.theta = req.theta;
     }
     else {
        HW::tturtles[i].turtlename = req.name;
        HW::tturtles[i].posetopic = tname.str();
        HW::tturtles[i].veltopic = vname.str();
        HW::tturtles[i].pose.x = req.x;
        HW::tturtles[i].pose.y = req.y;
        HW::tturtles[i].pose.theta = req.theta;
     };

     //if this turtle does not exist, create one else teleport it.
     if (!turtleExist(req.name.c_str())) {
        success = HW::sClient.call(req, resp);
        if(success) {
           if (tType == 'X')
              ROS_INFO("%s landed with face down.", req.name.c_str()); //X turtle
           else 
              ROS_INFO("%s landed with face up.", req.name.c_str()); //T turtle
        }
        else { 
          ROS_ERROR_STREAM("Error: Failed to create " << tType << " turtle.");
          ros::shutdown();
        }
     }
     else {
        cmdstr.clear();
        cmdstr.str("");
        cmdstr << "rosservice call /";
        cmdstr << req.name.c_str() << "/teleport_absolute " << req.x << " " << req.y << " " << req.theta;
        system(cmdstr.str().c_str()); 
        ROS_INFO_STREAM(req.name.c_str() << " already landed, so it's teleported!\n");
     };
  };
} 

double getDistance(const double x1, const double y1, const double x2, const double y2) {
  return sqrt(pow((x1-x2),2) + pow(y1-y2, 2));
}

bool isTooClose(double x1, double y1, double x2, double y2, double threshhold) {
  if (HW::getDistance(x1, y1, x2, y2) <= threshhold)
     return true;
  else
     return false;
}

bool isOffBoundary(double x, double y) {
  if (x <= LOWER_LIMIT || x > UPPER_LIMIT || y <= LOWER_LIMIT || y > UPPER_LIMIT)
     return true;
  else 
     return false;
}

void removeTurtle1() {
  turtlesim::Kill::Request reqk;
  turtlesim::Kill::Response respk;

  reqk.name = HW::turtle1.turtlename;
  if (!HW::kClient.call(reqk, respk))
     ROS_ERROR_STREAM("Error: Failed to kill " << reqk.name.c_str() << "\n");
  else
     ROS_INFO_STREAM("!!! Mission failed !!!");

  ROS_INFO_STREAM("...shutting down...\n");
  ros::shutdown();
}

void moveDistance(const double speed, const double distance, const bool isForward, HW::Turtle& turtle, const ros::Publisher& velpublisher) {

  geometry_msgs::Twist velmsg;

  if (isForward)
     velmsg.linear.x = abs(speed);
  else
     velmsg.linear.x = -abs(speed);

  velmsg.linear.y = 0;
  velmsg.linear.z = 0;
  velmsg.angular.x = 0;
  velmsg.angular.y = 0;
  velmsg.angular.z = 0;

  double t0 = ros::Time::now().toSec();
  double t1;
  double current_distance = 0.0;
  ros::Rate loop_rate(LOOP_SPEED);

  do{
     velpublisher.publish(velmsg);
     t1 = ros::Time::now().toSec();
     current_distance = speed * (t1-t0); //distance = delta_time*speed
     ros::spinOnce();
     loop_rate.sleep();
cout << turtle.turtlename << ":" << turtle.pose.x << "," << turtle.pose.y << "," << turtle.pose.theta << endl;
     if (isOffBoundary(turtle.pose.x, turtle.pose.y)) {
        //cout << " .revx.: " << velmsg.linear.x << endl;
        //cout << "pose: (" << turtle.pose.x << "," << turtle.pose.y << ")" << endl;
        velmsg.linear.x = -1*velmsg.linear.x;
        velpublisher.publish(velmsg);
        sleep(1);
     };
  } while (current_distance < distance);

  velmsg.linear.x =0;
  velpublisher.publish(velmsg);
  cout << " deltaTime=" << (t1-t0) << endl;
}

void rotate (const double angular_speed, const double relative_angle, const bool clockwise, const ros::Publisher& velpublisher){
  geometry_msgs::Twist velmsg;
  velmsg.linear.x = 0;
  velmsg.linear.y = 0;
  velmsg.linear.z = 0;
  velmsg.angular.x = 0;
  velmsg.angular.y = 0;

  if (clockwise)
     velmsg.angular.z =-abs(angular_speed);
  else
     velmsg.angular.z =abs(angular_speed);

  double current_angle = 0.0;
  double t0 = ros::Time::now().toSec();
  ros::Rate loop_rate(LOOP_SPEED);

  do{
     velpublisher.publish(velmsg);
     double t1 = ros::Time::now().toSec();
     current_angle = angular_speed * (t1-t0);
     ros::spinOnce();
     loop_rate.sleep();
  } while (current_angle < relative_angle);

  velmsg.angular.z = 0;
  velpublisher.publish(velmsg);
}

double degrees2radians(const double angle_degrees){
  return angle_degrees*PI/180.0;
}

void setDesiredOrientation (const double desired_angle_radians, const HW::Turtle& t, const ros::Publisher& velpublisher){
  double relative_angle_radians = desired_angle_radians - t.pose.theta;
  bool clockwise = ((relative_angle_radians<0)?true:false);

//cout<<desired_angle_radians <<","<<t.pose.theta<<","<<relative_angle_radians<<","<<clockwise<<endl;

  rotate (degrees2radians(ANGULAR_SPEED), abs(relative_angle_radians), clockwise, velpublisher);
}

void moveGoal(const turtlesim::Pose goal_pose, const double distance_tolerance, const HW::Turtle& t, const ros::Publisher& velpublisher){

  geometry_msgs::Twist velmsg;
  ros::Rate loop_rate(LOOP_SPEED);
  double E = 0.0;

cout << "Goal: " << goal_pose.x << "," << goal_pose.y << "," << goal_pose.theta << endl;
cout << t.turtlename << ": " << t.pose.x << "," << t.pose.y << "," << t.pose.theta << endl;

  do{
    //proportional controller
    double e = getDistance(t.pose.x, t.pose.y, goal_pose.x, goal_pose.y);
    double E = E+e;

    //Kp = v0 * (exp(-alpha)*error*error)/(error*error);
    velmsg.linear.x = (Kp*E);
    velmsg.linear.y = 0;
    velmsg.linear.z = 0;

    //angular velocity in the z-axis
    velmsg.angular.x = 0;
    velmsg.angular.y = 0;
    velmsg.angular.z = 4*(atan2(goal_pose.y-t.pose.y, goal_pose.x-t.pose.x)-t.pose.theta);

    velpublisher.publish(velmsg);
    ros::spinOnce();
    loop_rate.sleep();
  } while(getDistance(t.pose.x, t.pose.y, goal_pose.x, goal_pose.y)>distance_tolerance);

  velmsg.linear.x = 0;
  velmsg.angular.z = 0;
  velpublisher.publish(velmsg);
  cout<<t.turtlename << " moved to the target location."<<endl;
}

}; //end of HW namespace
///////////////////////////////////////

class Turtle1Listener {
  public:
    void doTest(const turtlesim::Pose::ConstPtr& msg); 
  private:
    bool chkIfOffBoundary();
    bool chkIsTooClose();
};

//turtle1 callback
void Turtle1Listener::doTest(const turtlesim::Pose::ConstPtr& msg) {
  //update turtle1 pose whenever turtle1 moves
  HW::turtle1.pose.x = msg->x;
  HW::turtle1.pose.y = msg->y;

  if (HW::turtle1MoveCnt == 0) {
     //cout << "turtle1Move cnt ++ \n";
     HW::turtle1Origin.x = msg->x;
     HW::turtle1Origin.y = msg->y;
     HW::turtle1MoveCnt++;
  } else if (HW::getDistance(HW::turtle1.pose.x, HW::turtle1.pose.y, HW::turtle1Origin.x, HW::turtle1Origin.y) > DANGER_TOLERANCE) { 
     HW::isTurtle1Moved = true;  
     //cout << "Turtle1 moved to (" << msg->x << "," << msg->y << ")\n";
  };

  //test case1
  if (chkIfOffBoundary())
     HW::removeTurtle1();

  //test case2
  if (chkIsTooClose())
     HW::removeTurtle1();
}; 

bool Turtle1Listener::chkIfOffBoundary() {
  if (HW::isOffBoundary(HW::turtle1.pose.x, HW::turtle1.pose.y)) {
     ROS_INFO_STREAM("turtle1 is moving off the limit at (" << HW::turtle1.pose.x << "," << HW::turtle1.pose.y << ")");
     return true;
  } else
     return false;
}

bool Turtle1Listener::chkIsTooClose() {
  int i;
  bool tooclose = false;
  double dist;

  //when turtle1 moves, check all X turtles' locations
  for (i=0; i<MAX_XTURTLES; i++) {
     dist = HW::getDistance(HW::turtle1.pose.x, HW::turtle1.pose.y, HW::xturtles[i].pose.x, HW::xturtles[i].pose.y);
     if (dist <= DANGER_TOLERANCE) {
        tooclose = true;
        ROS_INFO_STREAM("Turtle1 was too close to " << HW::xturtles[i].turtlename << " with distance = " << dist);
        ROS_INFO_STREAM("turtle1 was captured.");
        break;
     };
  };
  return tooclose;
}

/// TTurtleListener ///
class TTurtleListener {
  public: 
    void doTest(const turtlesim::Pose::ConstPtr& msg, const string turtlename); 
  private:
    bool isTooClose(int ti);
};

//tturtle callback
void TTurtleListener::doTest(const turtlesim::Pose::ConstPtr& msg, const string turtlename) {
  int turtleIdx;
  //update a tturtle pose whenever tturtle moves
  turtleIdx = atoi(turtlename.substr(1).c_str()); //extract turtle # from turtlename
  turtleIdx = turtleIdx - 1; //since index starts from 0
  HW::tturtles[turtleIdx].pose.x = msg->x;
  HW::tturtles[turtleIdx].pose.y = msg->y;

//check this logic???
  if (isTooClose(turtleIdx)) {
     ;//HW::removeTurtle1();
  };
}; 

//when a tturtle moves, check the turtle1's location
bool TTurtleListener::isTooClose(int ti) {
  double dist;
  bool tooclose = false;

  dist = HW::getDistance(HW::tturtles[ti].pose.x, HW::tturtles[ti].pose.y, HW::turtle1.pose.x, HW::turtle1.pose.y);
  if (dist <= DANGER_TOLERANCE) { 
     tooclose = true;
     ROS_INFO_STREAM(HW::tturtles[ti].turtlename << " is too close with distance = " << dist);
     ROS_INFO_STREAM("It may be captured by turtle1.");
  };
  return tooclose;
}

/// XTurtleListener ///
class XTurtleListener {
  public: 
    void doTest(const turtlesim::Pose::ConstPtr& msg, const string turtlename); 
  private:
    bool isTooClose(int ti);
};

//xturtle callback
void XTurtleListener::doTest(const turtlesim::Pose::ConstPtr& msg, const string turtlename) {
  int turtleIdx;
  //update a xturtle pose whenever xturtle moves
  turtleIdx = atoi(turtlename.substr(1).c_str()); //extract turtle # from turtlename
  turtleIdx = turtleIdx - 1; //since index starts from 0
  HW::xturtles[turtleIdx].pose.x = msg->x;
  HW::xturtles[turtleIdx].pose.y = msg->y;

  if (isTooClose(turtleIdx)) {
     HW::removeTurtle1();
  };
}; 

//when a xturtle moves, check the turtle1's location
bool XTurtleListener::isTooClose(int ti) {
  double dist;
  bool tooclose = false;

  dist = HW::getDistance(HW::xturtles[ti].pose.x, HW::xturtles[ti].pose.y, HW::turtle1.pose.x, HW::turtle1.pose.y);
  if (dist <= DANGER_TOLERANCE) { 
     tooclose = true;
     ROS_INFO_STREAM("Turtle1 was too close to " << HW::xturtles[ti].turtlename << " with distance = " << dist);
     ROS_INFO_STREAM("turtle1 was captured.");
  };
  return tooclose;
};

/// Turtles Behavior ///
class TurtlesBehavior {
  public: 
    TurtlesBehavior(ros::Publisher* tpubs, ros::Publisher* xpubs) {
      _tpubs = tpubs;
      _xpubs = xpubs;
    };

    void begin() {
      int loopCnt = 1; //for various distances
      int speed = rand() % 2 + 1;
      int rdir = rand() % 3; //for various rotations
      int distance = rand() % 20 + 1;
      ros::Rate loop_rate(LOOP_SPEED); 

      while (ros::ok()) {
        if (loopCnt >= 10) 
           loopCnt=1; 
        movePattern1(HW::tturtles[0], _tpubs[0], speed, distance); //T1
        movePattern1(HW::xturtles[0], _xpubs[0], speed, distance); //X1
        movePattern1(HW::xturtles[1], _xpubs[1], speed, distance); //X2
        movePattern2(HW::tturtles[1], _tpubs[1], speed, distance, loopCnt, HW::xturtles[2], _xpubs[2]); //T2 
        movePattern3(HW::tturtles[2], _tpubs[2], speed, distance, rdir); //T3 
        loop_rate.sleep();
        loopCnt++;
      };
    };
  private: 
    ros::Publisher* _tpubs;
    ros::Publisher* _xpubs;
    void movePattern1(HW::Turtle& turtle, ros::Publisher& pub, int speed, int distance);
    void movePattern2(HW::Turtle& tturtle, ros::Publisher& tpub, int speed, int distance, int loopCnt, HW::Turtle& xturtle, ros::Publisher& xpub);
    void movePattern3(HW::Turtle& tturtle, ros::Publisher& tpub, int speed, int distance, int rdir);
};

void TurtlesBehavior::movePattern1(HW::Turtle& turtle, ros::Publisher& pub, int speed, int distance) {
  if (HW::turtleExist(turtle.turtlename)) { //is this turtle alive?
     cout << "P1: " << turtle.turtlename << " distance=" << distance;
     cout << " speed=" << speed;
     HW::moveDistance(speed, distance, true, turtle, pub);
  };
};

void TurtlesBehavior::movePattern2(HW::Turtle& tturtle, ros::Publisher& tpub, 
     int speed, int distance, int loopCnt, HW::Turtle& xturtle, ros::Publisher& xpub) {
  turtlesim::Pose goal_pose;
  goal_pose.x = HW::turtle1.pose.x;
  goal_pose.y = HW::turtle1.pose.y;
  goal_pose.theta = HW::turtle1.pose.theta;

  distance = distance * ((loopCnt % 2)+1);

  if (HW::turtleExist(tturtle.turtlename)) { //is this turtle alive?
     cout << "P2: " << loopCnt << " " << tturtle.turtlename << " distance=" << distance;
     cout << " speed=" << speed;
     HW::moveDistance(speed, distance, true, tturtle, tpub);
     //xturtle can chase after turtle1 later
  };
};

void TurtlesBehavior::movePattern3(HW::Turtle& tturtle, ros::Publisher& tpub, int speed, int distance, int rdir) {

  double desired_angle_radians;

  if (rdir == 0)
     desired_angle_radians = PI;
  else if (rdir == 1)
     desired_angle_radians = -PI;
  else
     desired_angle_radians = 2*PI;

  if (HW::turtleExist(tturtle.turtlename)) { //is this turtle alive?
     cout << "P3: " << tturtle.turtlename << " distance=" << distance;
     cout << " speed=" << speed;
     cout << " rotationAngle= " << desired_angle_radians;
     HW::moveDistance(speed, distance, true, tturtle, tpub);
     setDesiredOrientation(desired_angle_radians, tturtle, tpub);
     HW::moveDistance(speed, distance, true, tturtle, tpub);
     setDesiredOrientation(desired_angle_radians, tturtle, tpub);
  };
};

////////////////
class HWTest {
  public: 
    HWTest(ros::NodeHandle* anh) {
      _nh = *anh;
    };

    void init();
    void startTest();

  private:
    ros::NodeHandle _nh; 
    ros::Subscriber _turtle1sub;
    ros::Subscriber _tturtlesubs[MAX_TTURTLES];
    ros::Subscriber _xturtlesubs[MAX_XTURTLES];
    Turtle1Listener _turtle1listener;
    TTurtleListener _tturtlelisteners[MAX_TTURTLES];
    XTurtleListener _xturtlelisteners[MAX_XTURTLES];
    ros::Publisher _turtle1pub;
    ros::Publisher _tturtlepubs[MAX_TTURTLES];
    ros::Publisher _xturtlepubs[MAX_XTURTLES];
};

void HWTest::init() {
  int i;
  stringstream cmdstr;

  HW::sClient = _nh.serviceClient<turtlesim::Spawn>("spawn");
  HW::kClient = _nh.serviceClient<turtlesim::Kill>("kill");

  //set seed for random number
  srand(time(0));

  HW::turtle1.turtlename = "turtle1";
  HW::turtle1.posetopic = "/turtle1/pose";
  HW::turtle1.veltopic = "/turtle1/cmd_vel";

  //create T turtles before X turtles to avoid landing xturtle on the same location
  HW::createTurtles('T', MAX_TTURTLES);
  HW::createTurtles('X', MAX_XTURTLES);

  //create turtle1 subsriber 
  _turtle1sub = _nh.subscribe<turtlesim::Pose>(HW::turtle1.posetopic, 1000, &Turtle1Listener::doTest, &_turtle1listener); 

  //create tturtle subsribers 
  for (i=0; i<MAX_TTURTLES; i++) {
     _tturtlesubs[i] = _nh.subscribe<turtlesim::Pose>(HW::tturtles[i].posetopic, 1000, boost::bind(&TTurtleListener::doTest, &_tturtlelisteners[i], _1, HW::tturtles[i].turtlename));
  };

  //create xturtle subsribers 
  for (i=0; i<MAX_XTURTLES; i++) {
     _xturtlesubs[i] = _nh.subscribe<turtlesim::Pose>(HW::xturtles[i].posetopic, 1000, boost::bind(&XTurtleListener::doTest, &_xturtlelisteners[i], _1, HW::xturtles[i].turtlename));
  };

  //create turtle1 publisher
  _turtle1pub = _nh.advertise<geometry_msgs::Twist>(HW::turtle1.veltopic, 1000);

  //create tturtle publishers 
  for (i=0; i<MAX_TTURTLES; i++) {
      cmdstr.clear();
      cmdstr.str("");
      cmdstr << "/T" << i+1;
      cmdstr << "/cmd_vel";
      HW::tturtles[i].veltopic = cmdstr.str();
      _tturtlepubs[i] = _nh.advertise<geometry_msgs::Twist>(HW::tturtles[i].veltopic, 1000);
  };

  //create xturtle publishers 
  for (i=0; i<MAX_XTURTLES; i++) {
      cmdstr.clear();
      cmdstr.str("");
      cmdstr << "/X" << i+1;
      cmdstr << "/cmd_vel";
      HW::xturtles[i].veltopic = cmdstr.str();
      _xturtlepubs[i] = _nh.advertise<geometry_msgs::Twist>(HW::xturtles[i].veltopic, 1000);
  };
}

void HWTest::startTest() {
  ROS_INFO_STREAM("---------------- Ready to Test ----------------");
  ROS_INFO_STREAM("1. turtle1 will be removed if it moves off the limit (" << LOWER_LIMIT << "," << LOWER_LIMIT << ") and (" << UPPER_LIMIT << "," << UPPER_LIMIT << ")");
  ROS_INFO_STREAM("2. turtle1 can capture T turtle within the distance " << DANGER_TOLERANCE);
  ROS_INFO_STREAM("3. X turtle will capture turtle1 within the distance " << DANGER_TOLERANCE);
  ROS_INFO_STREAM("As soon as turtle1 starts moving, T and X turtles will move."); 
  ROS_INFO_STREAM("-----------------------------------------------");

  //create a turtle behavior object
  TurtlesBehavior tb(_tturtlepubs, _xturtlepubs);

  ros::Rate loop_rate(LOOP_SPEED);

  while (ros::ok()) {
     ros::spinOnce(); //this is necessary for subscribing topics
     loop_rate.sleep();

     if (HW::isTurtle1Moved) { 
        // TurtlesBehavior class could have been created as a library 
        // to hide the implementation of behavior patterns in begin() method.
        tb.begin(); 
        break;
     }; //else we will wait until turtle1 moves
  };
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "HWTest4");
  ros::NodeHandle nh;

  HWTest hw4t(&nh);
  hw4t.init();
  hw4t.startTest();
  return 0;
}

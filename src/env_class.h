/*
 Node: env_class 
 Author: German Ruiz Mudarra, May 2020

 Description: Environment Classifier
    
 Subscriptions:
 /curvature_calc/all (Information about curvature of the lanes)
 /odom (Global position of the car (not ground-truth))

 Publications:

 */
#ifndef ENV_CLASS_H
#define ENV_CLASS_H

#include "ros/ros.h"

#include "iostream"
using namespace std;

#include "math.h"

#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/String.h"
#include "adir/curvature_t.h"
#include "adir/enable_t.h"

// Cartesian coordinates
struct position_t {
    double x;
    double y;
    // Constructor for easy mapping
    position_t(double arg_x, double arg_y) : x(arg_x), y(arg_y) {}
};

// Subscriptions 


static const uint32_t CURV_QUEUE_SIZE = 1000;
static const uint32_t ODOM_QUEUE_SIZE = 1;
// Publications 

// Message queueing parameters
static const uint32_t LD_QUEUE_SIZE = 1000;
static const uint32_t PLANNING_QUEUE_SIZE = 1000;
static const uint32_t SPEED_QUEUE_SIZE = 1000;


static const uint32_t CURV_THRESHOLD = 0.1;

void callbackCurvData(const adir::curvature_t::ConstPtr& msg);
void callbackOdomData(const nav_msgs::Odometry::ConstPtr& msg);

double getDistance(position_t p, position_t q);
std::vector<position_t> defineIntersectionNodes();
bool checkPosition(std::vector<position_t> map, int& node_id);
string intersectionClassifier(int node_id);
void environmentClassifier(std::vector<position_t> map);


int loop_rate;
double lookahead;
int stop_speed;
int move_speed;
string curvature_topic;
string odometry_topic;
string line_detection_topic;
string planning_topic;
string speed_topic;


#endif

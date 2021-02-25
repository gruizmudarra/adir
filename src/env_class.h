/*
 Node: env_class 
 Author: German Ruiz Mudarra, UMA Garage Team.
 Created: May 2020
 Last modified: February 2021

 Description: This node uses odometry information and the curvature of the lanes recorded by the front camera of the
 car to classify the environment where it is located on the urban circuit. 
 
 Odometry is compared with a topologic map where crossing/roundabout entries/exits are defined as nodes. 
 If the car is near one of these nodes (euclidean distance), the classifier will activate the specific planning for
 these scenarios. 
 On the other hand, if the car is not near a node, it means its not going to face a intersection, so the lane follower
 should be activated (if not yet).   
    
 Subscriptions:
 /adir/curvature_calc/all (Information about curvature of the lanes)
 /odom (Estimated global position of the car)

 Publications:


Subscriptions:
    /odom                   (nav_msgs::Odometry): Degree of the polynomials detected
    /adir/curvature_calc    (std_msgs::Float32Multiarray): Vector with the maximum curvature of each lane

Publications:
    /adir/
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
static const uint32_t ENV_QUEUE_SIZE = 1;


static const double CURV_THRESHOLD = 0.007;

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
string environment_topic;
string speed_topic;


#endif

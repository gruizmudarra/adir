/*
 Node: roundabout_planner 
 Author: German Ruiz Mudarra, July 2020

 Description: Roundabout environment path planning.
    
 Subscriptions:


 Publications:

 */
#ifndef ROUNDABOUT_PLANNER_H
#define ROUNDABOUT_PLANNER_H
#include "ros/ros.h"

#include "iostream"
using namespace std;

#include "math.h"

#include "nav_msgs/Odometry.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"

#include "adir/enable_t.h"
#include "adir/point2D.h"

#define PRINT_MARKERS

// Cartesian coordinates
struct position_t {
    double x;
    double y;
    // Constructor for easy mapping
    position_t(double arg_x, double arg_y) : x(arg_x), y(arg_y) {}
};

enum maneuver_state_t {IDLE_STATE, DEFINITION_STATE, ENTRY_STATE, CIRCULATION_STATE, EXIT_STATE};


static const double TRANSIT_RADIUS = 1.6; // Distance between road and center of the roundabout




// Subscription callbacks
void callbackOdomData(const nav_msgs::Odometry::ConstPtr& msg);
void callbackADIRData(const adir::enable_t::ConstPtr& msg);

// Bezier curves calculation
double bezierLinearScalar(double a, double b, double t);
position_t bezierLinear(position_t P, position_t Q, double t);
position_t bezierQuartic(position_t P, position_t Q, position_t R, position_t S, position_t U, double t);
position_t circunference(position_t P, position_t c, double r, double t);
// 
double getDistance(position_t p, position_t q);
position_t getVector(position_t p, position_t q);
position_t getUnitVector(position_t p, position_t q);

void selectRestrictionPoints();
void defineControlPoints();
void printMarkers();
void printReference(position_t point);
void publishReference(position_t r);
void roundaboutReferenceGenerator();

int loop_rate;
double lookahead;

string odometry_topic;
string planning_topic;

string reference_topic;
string planning_markers_topic;
string control_topic;
string speed_topic;

static const uint32_t ODOM_QUEUE_SIZE = 1;
static const uint32_t ENV_QUEUE_SIZE = 1;
static const uint32_t ENABLE_QUEUE_SIZE = 1;
static const uint32_t PLANNING_QUEUE_SIZE = 1;
static const uint32_t MARKERS_QUEUE_SIZE = 20;
#endif

/*
 Node: crossing_planner 
 Author: German Ruiz Mudarra, July 2020

 Description: Crossing environment path planning.
    
 Subscriptions:


 Publications:

 */
#ifndef CROSSING_PLANNER_H
#define CROSSING_PLANNER_H
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

enum maneuver_state_t {IDLE, DEFINITION_STATE, CIRCULATION_STATE};

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
void crossingReferenceGenerator();

int loop_rate;
double lookahead;
#endif
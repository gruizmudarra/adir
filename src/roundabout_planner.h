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

#define PRINT_MARKERS

// Cartesian coordinates
struct position_t {
    double x;
    double y;
    // Constructor for easy mapping
    position_t(double arg_x, double arg_y) : x(arg_x), y(arg_y) {}
};

enum maneuver_state_t {IDLE, DEFINITION_STATE, ENTRY_STATE, CIRCULATION_STATE, EXIT_STATE};

// Subscription callbacks
void cb_odomData(const nav_msgs::Odometry::ConstPtr& msg);
void cb_envData(const std_msgs::String::ConstPtr& msg);
void cb_enableData(const std_msgs::Bool::ConstPtr& msg);
void cb_nodeData(const std_msgs::Int16::ConstPtr& msg);

// Bezier curves calculation
double bezier_linear_scalar(double a, double b, double t);
position_t bezier_linear(position_t P, position_t Q, double t);
position_t bezier_quartic(position_t P, position_t Q, position_t R, position_t S, position_t U, double t);
position_t circunference(position_t P, position_t c, double r, double t);
// 
double get_distance(position_t p, position_t q);
position_t get_vector(position_t p, position_t q);
position_t get_unit_vector(position_t p, position_t q);

void select_restriction_points();
void define_control_points();
void print_markers();
void print_reference(position_t point);
void roundabout_reference_generator();
#endif

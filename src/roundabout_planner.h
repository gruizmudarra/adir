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
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "math.h"

using namespace std;

// Cartesian coordinates
struct position_t {
    double x;
    double y;
    // Constructor for easy mapping
    position_t(double arg_x, double arg_y) : x(arg_x), y(arg_y) {}
};

// Subscription callbacks
void cb_odomData(const nav_msgs::Odometry::ConstPtr& msg);
void cb_envData(const std_msgs::String::ConstPtr& msg);
void cb_enableData(const std_msgs::Bool::ConstPtr& msg);

// Bezier curves calculation
double bezier_linear_scalar(double a, double b, double t);
position_t bezier_linear(position_t p1, position_t p2, double t);
position_t bezier_quartic(position_t p1, position_t p2, position_t p3, position_t p4, position_t p5, double t);

// 
double get_distance(position_t p, position_t q);
position_t get_vector(position_t p, position_t q);
position_t get_unit_vector(position_t p, position_t q);
void select_restriction_points(position_t& ientry1, position_t& ientry2, position_t& iexit1, position_t& iexit2, position_t& rexit)
void roundabout_reference_generator();

#endif
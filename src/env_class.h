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
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/String.h"
#include "iostream"
using namespace std;

struct curvature_t {
    float left;
    float center;
    float right;
};

// Cartesian coordinates
struct position_t {
    double x;
    double y;
    // Constructor for easy mapping
    position_t(double arg_x, double arg_y) : x(arg_x), y(arg_y) {}
};

void cb_curvData(const std_msgs::Float32MultiArray::ConstPtr& msg);
void cb_odomData(const nav_msgs::Odometry::ConstPtr& msg);
float get_distance(float x, float y);
std::vector<position_t> define_intersection_nodes();
void environment_classifier(std::vector<position_t> map);
bool check_position(std::vector<position_t> map, int& node_id);
string intersection_class(int node_id);

#endif

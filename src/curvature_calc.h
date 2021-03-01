/*
 Node: curvature_calc
 Author: German Ruiz Mudarra, UMA Garage Team.
 Created: April 2020
 Last modified: February 2021

 Description:
    When "line_detection_fu_node" detects a certain number of points on a lane of the road, 
    it adjust a parabola (ax²+bx+c).
    This node receives the polynomial information and calculates the radius of curvature.
    If the polynomial seems like a straight line, it will have a very low "a" value and the curvature will tend to zero. 
    If the polynomial begins to have some curvature, then the curvature will have a fixed value.
    If no line is detected, the curvature will have a very high value.
    Once the curvature is calculated, it is sent to the environment classifier, which will use this information to do its job.

 Subscriptions:
    /lane_model/poly_degrees    (std_msgs::Float32Multiarray): Degree of the polynomials detected
    /lane_model/coef/Left       (std_msgs::Float32Multiarray): [c,b,a] coefficients of the left lane polynomial (usually not available)
    /lane_model/coef/Center     (std_msgs::Float32Multiarray): [c,b,a] coefficients of the center lane polynomial
    /lane_model/coef/Right      (std_msgs::Float32Multiarray): [c,b,a] coefficients of the right lane polynomial

Publications:
    /adir/curvature_calc        (std_msgs::Float32Multiarray): Vector with the maximum curvature of each lane
*/

#ifndef CURVATURE_CALC_H
    #define CURVATURE_CALC_H
    // #define PLOT_CURVATURE_DATA // Macro for data debugging
    #include "ros/ros.h"
    #include "iostream"
    #include "math.h"
    using namespace std;

    // Import ROS message libraries
    #include "std_msgs/Int32MultiArray.h"
    #include "std_msgs/Float32MultiArray.h"
    #include "std_msgs/Float32.h"

    // Import ADIR custom message for curvature
    #include "adir/curvature_t.h"

    // Message queueing parameters
    static const uint32_t POLY_QUEUE_SIZE = 1;
    static const uint32_t CURV_QUEUE_SIZE = 1000;

    // Topics for debugging data
    #ifdef PLOT_CURVATURE_DATA
        static const string CURVATURE_TOPIC_LEFT = "/adir/curvature_calc/left";
        static const string CURVATURE_TOPIC_CENTER = "/adir/curvature_calc/center";
        static const string CURVATURE_TOPIC_RIGHT = "/adir/curvature_calc/right";
    #endif

    // This struct define the polynomials parameters
    struct polynomial_t {
        double a;
        double b;
        double c;
        int degree;
    };

    // Function promises
    void callbackCoefLeft(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackCoefCenter(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackCoefRight(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackDegrees(const std_msgs::Int32MultiArray::ConstPtr& msg);
    void curvatureCalculation();

    // Variables 
    #ifdef PLOT_CURVATURE_DATA
        ros::Publisher left_pub;
        std_msgs::Float32 left_msg;

        ros::Publisher center_pub;
        std_msgs::Float32 center_msg;

        ros::Publisher right_pub;
        std_msgs::Float32 right_msg;
    #endif

    // Variables for subscribing and storing polynomial data 
    ros::Subscriber degrees_sub, coefLeft_sub, coefCenter_sub, coefRight_sub;
    polynomial_t LeftLane, CenterLane, RightLane;

    // Variables for publishing curvature
    adir::curvature_t curvature_msg;
    ros::Publisher curvature_pub;

    // Variables for ROS parameters
    int loop_rate;
    string curvature_topic;
#endif
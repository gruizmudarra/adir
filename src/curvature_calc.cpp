/*
 Node: curvature_calc
 Author: German Ruiz Mudarra, April 2020

 Description:
    When "line_detection_fu_node" detects a certain number of points on a lane of the road, 
    it adjust a polynomial, usually a parabola (ax²+bx+c).
    This node receives the polynomial information and calculates the radius of curvature.
    If the polynomial seems like a straight line, it will have a very low "a" value and the radius of curvature will tend to infinity. 
    If the polynomial begins to have some curvature, then the radius of curvature will have a fixed value.

 Subscriptions:
    /lane_model/poly_degrees (Float32Multiarray): Degree of the polynomials detected
    /lane_model/coef/Left   (Float32Multiarray): [...,c,b,a] coefficients of the polynomials (Usually a parabola with 3 coefficients)
    /lane_model/coef/Center
    /lane_model/coef/Right

Publications:
    /curvature_calc/left    (Float32): Maximum radius of curvature o the polynomial.
    /curvature_calc/center
    /curvature_calc/right

 */

#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"

#define PLOT_CURVATURE_DATA

static const uint32_t MY_ROS_QUEUE_SIZE = 1;

using namespace std;

void cb_coefLeft(const std_msgs::Float32MultiArray::ConstPtr& msg);
void cb_coefCenter(const std_msgs::Float32MultiArray::ConstPtr& msg);
void cb_coefRight(const std_msgs::Float32MultiArray::ConstPtr& msg);
void cb_degrees(const std_msgs::Int32MultiArray::ConstPtr& msg);
void curvature_calculation();

ros::Publisher curvature_pub;
std_msgs::Float32MultiArray sent_message;

#ifdef PLOT_CURVATURE_DATA
ros::Publisher left_pub;
std_msgs::Float32 left_msg;

ros::Publisher center_pub;
std_msgs::Float32 center_msg;

ros::Publisher right_pub;
std_msgs::Float32 right_msg;
#endif

struct polynomial_t {
    float a;
    float b;
    float c;
    int degree;
    float curvature;
};

polynomial_t LeftLane, CenterLane, RightLane;
float LeftCurvature, CenterCurvature, RightCurvature;

 int main (int argc, char **argv) {
    
    ros::init(argc, argv, "curvature_calc");

	ros::NodeHandle nh;
    // Subscribe to polynomials degree data
    ros::Subscriber degrees_sub = nh.subscribe("/lane_model/poly_degrees", MY_ROS_QUEUE_SIZE, cb_degrees);
    // Subscribe to polynomials coefficients data
    ros::Subscriber coefLeft_sub = nh.subscribe("/lane_model/coef/Left", MY_ROS_QUEUE_SIZE, cb_coefLeft);
    ros::Subscriber coefCenter_sub = nh.subscribe("/lane_model/coef/Center", MY_ROS_QUEUE_SIZE, cb_coefCenter);
    ros::Subscriber coefRight_sub = nh.subscribe("/lane_model/coef/Right", MY_ROS_QUEUE_SIZE, cb_coefRight);
    
    //Publications
    
    #ifdef PLOT_CURVATURE_DATA
    left_pub = nh.advertise<std_msgs::Float32>("/curvature_calc/left", MY_ROS_QUEUE_SIZE);
    center_pub = nh.advertise<std_msgs::Float32>("/curvature_calc/center", MY_ROS_QUEUE_SIZE);
    right_pub = nh.advertise<std_msgs::Float32>("/curvature_calc/right", MY_ROS_QUEUE_SIZE);
    #endif
    
    curvature_pub = nh.advertise<std_msgs::Float32MultiArray>("/curvature_calc/all", MY_ROS_QUEUE_SIZE);
    
    while (ros::ok()) {
        // Clear array publication
        sent_message.data.clear();

        // Calculate curvature radius      
        curvature_calculation();

        // Publish data
        
        #ifdef PLOT_CURVATURE_DATA
        left_pub.publish(left_msg);
        center_pub.publish(center_msg);
        right_pub.publish(right_msg);
        #endif
        
        curvature_pub.publish(sent_message);
        
        ros::spinOnce();
        sleep(1);
    }
    return 0;
 }

void cb_coefLeft(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    std::vector<float> temp_data;
    for (int i=0; i < msg-> data.size(); i++) {
        temp_data.push_back(msg -> data[i]);
    }
    if (temp_data.size() == 3) {
        LeftLane.a = temp_data[2];
        LeftLane.b = temp_data[1];
        LeftLane.c = temp_data[0];
    }
    if (temp_data.size() == 2) {
        LeftLane.a = 0;
        LeftLane.b = temp_data[1];
        LeftLane.c = temp_data[0];
    }
    if (temp_data.size() == 1) {
        LeftLane.a = 0;
        LeftLane.b = 0;
        LeftLane.c = temp_data[0];
    }
    ROS_INFO("Left Coef data saved");
}

void cb_coefCenter(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    std::vector<float> temp_data;
    for (int i=0; i < msg-> data.size(); i++) {
        temp_data.push_back(msg -> data[i]);
    }
    if (temp_data.size() == 3) {
        CenterLane.a = temp_data[2];
        CenterLane.b = temp_data[1];
        CenterLane.c = temp_data[0];
    }
    if (temp_data.size() == 2) {
        CenterLane.a = 0;
        CenterLane.b = temp_data[1];
        CenterLane.c = temp_data[0];
    }
    if (temp_data.size() == 1) {
        CenterLane.a = 0;
        CenterLane.b = 0;
        CenterLane.c = temp_data[0];
    }
    ROS_INFO("Center Coef data saved");
}

void cb_coefRight(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    // Extract data to a temporary variable
    std::vector<float> temp_data;
    for (int i=0; i < msg-> data.size(); i++) {
        temp_data.push_back(msg -> data[i]);
    }
    // Second grade polynomial
    if (temp_data.size() == 3) {
        RightLane.a = temp_data[2];
        RightLane.b = temp_data[1];
        RightLane.c = temp_data[0];
    }
    // First grade polynomial
    if (temp_data.size() == 2) {
        RightLane.a = 0;
        RightLane.b = temp_data[1];
        RightLane.c = temp_data[0];
    }
    // Constant or not detected
    if (temp_data.size() == 1) {
        RightLane.a = 0;
        RightLane.b = 0;
        RightLane.c = temp_data[0];
    }
    ROS_INFO("Right Coef data saved");
}

void cb_degrees(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    // Extract data to a temporary variable
    std::vector<int> temp_data;
    for (int i=0; i < msg -> data.size(); i++) {
        temp_data.push_back(msg -> data[i]);
    }

    LeftLane.degree = temp_data[0];
    CenterLane.degree = temp_data[1];
    RightLane.degree = temp_data[2];
    ROS_INFO("Degree data saved");
}

void curvature_calculation() {
    //Curvature radius is defined by
    /*
    *   R_c = ((1+(df/dx)²)^3/2)/|d²f/dx| 
    */
    
    // Second grade curvature radius is then: 
    /*
    *   R_c = 1/(2*a)
    */

    // Left Lane
    if(LeftLane.degree == 2) {
        LeftLane.curvature = 1/(2*LeftLane.a);
        sent_message.data.push_back(LeftLane.curvature);
        
        #ifdef PLOT_CURVATURE_DATA
        left_msg.data = LeftLane.curvature;
        #endif
    }
    else {
        //ROS_INFO("Couldn't calculate Left Lane curvature");
        sent_message.data.push_back(0);
        #ifdef PLOT_CURVATURE_DATA
            left_msg.data = 0;
        #endif
    }

    // Center Lane
    if(CenterLane.degree == 2) {
        CenterLane.curvature = 1/(2*CenterLane.a);
        sent_message.data.push_back(CenterLane.curvature);
        #ifdef PLOT_CURVATURE_DATA
            center_msg.data = CenterLane.curvature;
        #endif
    }
    else {
        //ROS_INFO("Couldn't calculate Center Lane curvature");
        sent_message.data.push_back(0);
        #ifdef PLOT_CURVATURE_DATA
            center_msg.data = 0;
        #endif
    }

    // Right Lane
    if(RightLane.degree == 2) {
        RightLane.curvature = 1/(2*RightLane.a);
        sent_message.data.push_back(RightLane.curvature);
        #ifdef PLOT_CURVATURE_DATA
            right_msg.data = RightLane.curvature;
        #endif
    }
    
    else {
        //ROS_INFO("Couldn't calculate Center Lane curvature");
        sent_message.data.push_back(0);
        #ifdef PLOT_CURVATURE_DATA
            right_msg.data = 0;
        #endif
    }

}

 

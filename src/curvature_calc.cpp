#include "curvature_calc.h"

#define PLOT_CURVATURE_DATA

static const uint32_t POLY_QUEUE_SIZE = 1;
static const uint32_t CURV_QUEUE_SIZE = 1000;

#ifdef PLOT_CURVATURE_DATA
ros::Publisher left_pub;
std_msgs::Float32 left_msg;

ros::Publisher center_pub;
std_msgs::Float32 center_msg;

ros::Publisher right_pub;
std_msgs::Float32 right_msg;
#endif

adir::curvature_t curvature_msg;

polynomial_t LeftLane, CenterLane, RightLane;

 int main (int argc, char **argv) {
    
    ros::init(argc, argv, "curvature_calc");

	ros::NodeHandle nh;
    //Get params
    nh.param<int>("/loop_rate", loop_rate,50);
    // Subscribe to polynomials degree data
    ros::Subscriber degrees_sub = nh.subscribe("/lane_model/deg", POLY_QUEUE_SIZE, callbackDegrees);
    // Subscribe to polynomials coefficients data
    ros::Subscriber coefLeft_sub = nh.subscribe("/lane_model/coef/Left", POLY_QUEUE_SIZE, callbackCoefLeft);
    ros::Subscriber coefCenter_sub = nh.subscribe("/lane_model/coef/Center", POLY_QUEUE_SIZE, callbackCoefCenter);
    ros::Subscriber coefRight_sub = nh.subscribe("/lane_model/coef/Right", POLY_QUEUE_SIZE, callbackCoefRight);
    
    //Publications
    
    #ifdef PLOT_CURVATURE_DATA
    left_pub = nh.advertise<std_msgs::Float32>("/curvature_calc/left", CURV_QUEUE_SIZE);
    center_pub = nh.advertise<std_msgs::Float32>("/curvature_calc/center", CURV_QUEUE_SIZE);
    right_pub = nh.advertise<std_msgs::Float32>("/curvature_calc/right", CURV_QUEUE_SIZE);
    #endif
    
    ros::Publisher curvature_pub = nh.advertise<adir::curvature_t>("/lane_curvature", CURV_QUEUE_SIZE);

    ros::Rate node_loop_rate(loop_rate);
    
    while (ros::ok()) {
        // Calculate curvature radius      
        curvatureCalculation();

        // Publish data
        #ifdef PLOT_CURVATURE_DATA
        left_pub.publish(left_msg);
        center_pub.publish(center_msg);
        right_pub.publish(right_msg);
        #endif
        curvature_pub.publish(curvature_msg);
        
        ros::spinOnce();
        node_loop_rate.sleep();
    }
    return 0;
 }

void callbackCoefLeft(const std_msgs::Float32MultiArray::ConstPtr& msg) {
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

void callbackCoefCenter(const std_msgs::Float32MultiArray::ConstPtr& msg) {
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

void callbackCoefRight(const std_msgs::Float32MultiArray::ConstPtr& msg) {
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

void callbackDegrees(const std_msgs::Int32MultiArray::ConstPtr& msg) {
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

void curvatureCalculation() {
    //Curvature radius is defined by
    // R_c = ((1+(df/dx)²)^3/2)/|d²f/dx| 
    
    // Second grade curvature radius is then: 
    // R_c = 1/(2*a)
    // rho = 2*a

    // Left Lane
    if(LeftLane.degree > 0) {
        curvature_msg.left = 2*LeftLane.a;
        #ifdef PLOT_CURVATURE_DATA
        left_msg.data = curvature_msg.left;
        #endif
    }
    else {
        //ROS_INFO("Couldn't calculate Left Lane curvature");
        curvature_msg.left = 1000;
        #ifdef PLOT_CURVATURE_DATA
            left_msg.data = 1000;
        #endif
    }
    // Center Lane
    if(CenterLane.degree > 0) {
       curvature_msg.center = 2*CenterLane.a;
        #ifdef PLOT_CURVATURE_DATA
            center_msg.data = curvature_msg.center;
        #endif
    }
    else {
        curvature_msg.center = 1000;
        #ifdef PLOT_CURVATURE_DATA
            center_msg.data = 1000;
        #endif
    }
    // Right Lane
    if(RightLane.degree > 0) {
        curvature_msg.right = 2*RightLane.a;
        #ifdef PLOT_CURVATURE_DATA
            right_msg.data = curvature_msg.right;
        #endif
    }
    else {
        curvature_msg.right = 1000;
        #ifdef PLOT_CURVATURE_DATA
            right_msg.data = 1000;
        #endif
    }
}

 

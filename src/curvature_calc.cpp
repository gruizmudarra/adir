#include "curvature_calc.h"

// #define PLOT_CURVATURE_DATA

static const uint32_t LOOP_RATE = 5; // Hz


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

ros::Publisher curvature_pub;
std_msgs::Float32MultiArray curvature_array;

polynomial_t LeftLane, CenterLane, RightLane;
float LeftCurvature, CenterCurvature, RightCurvature;

 int main (int argc, char **argv) {
    
    ros::init(argc, argv, "curvature_calc");

	ros::NodeHandle nh;
    // Subscribe to polynomials degree data
    ros::Subscriber degrees_sub = nh.subscribe("/lane_model/deg", POLY_QUEUE_SIZE, cb_degrees);
    // Subscribe to polynomials coefficients data
    ros::Subscriber coefLeft_sub = nh.subscribe("/lane_model/coef/Left", POLY_QUEUE_SIZE, cb_coefLeft);
    ros::Subscriber coefCenter_sub = nh.subscribe("/lane_model/coef/Center", POLY_QUEUE_SIZE, cb_coefCenter);
    ros::Subscriber coefRight_sub = nh.subscribe("/lane_model/coef/Right", POLY_QUEUE_SIZE, cb_coefRight);
    
    //Publications
    
    #ifdef PLOT_CURVATURE_DATA
    left_pub = nh.advertise<std_msgs::Float32>("/curvature_calc/left", CURV_QUEUE_SIZE);
    center_pub = nh.advertise<std_msgs::Float32>("/curvature_calc/center", CURV_QUEUE_SIZE);
    right_pub = nh.advertise<std_msgs::Float32>("/curvature_calc/right", CURV_QUEUE_SIZE);
    #endif
    
    curvature_pub = nh.advertise<std_msgs::Float32MultiArray>("/curvature_calc/array", CURV_QUEUE_SIZE);

    ros::Rate node_loop_rate(LOOP_RATE);
    
    while (ros::ok()) {
        // Clear array publication
        curvature_array.data.clear();

        // Calculate curvature radius      
        curvature_calculation();

        // Publish data
        #ifdef PLOT_CURVATURE_DATA
        left_pub.publish(left_msg);
        center_pub.publish(center_msg);
        right_pub.publish(right_msg);
        #endif
        curvature_pub.publish(curvature_array);
        
        ros::spinOnce();
        node_loop_rate.sleep();
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
    // R_c = ((1+(df/dx)²)^3/2)/|d²f/dx| 
    
    // Second grade curvature radius is then: 
    // R_c = 1/(2*a)

    // Left Lane
    if(LeftLane.degree > 0) {
        LeftLane.curvature = 1/(2*LeftLane.a);
        curvature_array.data.push_back(LeftLane.curvature);
        
        #ifdef PLOT_CURVATURE_DATA
        left_msg.data = LeftLane.curvature;
        #endif
    }
    else {
        //ROS_INFO("Couldn't calculate Left Lane curvature");
        curvature_array.data.push_back(0);
        #ifdef PLOT_CURVATURE_DATA
            left_msg.data = 0;
        #endif
    }
    // Center Lane
    if(CenterLane.degree > 0) {
        CenterLane.curvature = 1/(2*CenterLane.a);
        curvature_array.data.push_back(CenterLane.curvature);
        #ifdef PLOT_CURVATURE_DATA
            center_msg.data = CenterLane.curvature;
        #endif
    }
    else {
        curvature_array.data.push_back(0);
        #ifdef PLOT_CURVATURE_DATA
            center_msg.data = 0;
        #endif
    }
    // Right Lane
    if(RightLane.degree > 0) {
        RightLane.curvature = 1/(2*RightLane.a);
        curvature_array.data.push_back(RightLane.curvature);
        #ifdef PLOT_CURVATURE_DATA
            right_msg.data = RightLane.curvature;
        #endif
    }
    else {
        curvature_array.data.push_back(0);
        #ifdef PLOT_CURVATURE_DATA
            right_msg.data = 0;
        #endif
    }
}

 

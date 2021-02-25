#include "curvature_calc.h"

 int main (int argc, char **argv) {
    
    ros::init(argc, argv, "curvature_calc"); // Init ROS node
	ros::NodeHandle nh; // Node handler
    
    // Get params needed
    nh.param<int>("/loop_rate", loop_rate,50); // Frequency of execution
    nh.param<string>("/curvature_topic", curvature_topic, "/adir/curvature_calc"); // Topic where curvature data is sent
    
    // Subscribe to polynomials degree topic
    ros::Subscriber degrees_sub = nh.subscribe("/lane_model/deg", POLY_QUEUE_SIZE, callbackDegrees);
    // Subscribe to polynomials coefficients topics
    ros::Subscriber coefLeft_sub = nh.subscribe("/lane_model/coef/Left", POLY_QUEUE_SIZE, callbackCoefLeft);
    ros::Subscriber coefCenter_sub = nh.subscribe("/lane_model/coef/Center", POLY_QUEUE_SIZE, callbackCoefCenter);
    ros::Subscriber coefRight_sub = nh.subscribe("/lane_model/coef/Right", POLY_QUEUE_SIZE, callbackCoefRight);
    
    // Publish to curvature topic
    ros::Publisher curvature_pub = nh.advertise<adir::curvature_t>(curvature_topic, CURV_QUEUE_SIZE);
    #ifdef PLOT_CURVATURE_DATA
        left_pub = nh.advertise<std_msgs::Float32>(CURVATURE_TOPIC_LEFT, CURV_QUEUE_SIZE);
        center_pub = nh.advertise<std_msgs::Float32>(CURVATURE_TOPIC_CENTER, CURV_QUEUE_SIZE);
        right_pub = nh.advertise<std_msgs::Float32>(CURVATURE_TOPIC_RIGHT, CURV_QUEUE_SIZE);
    #endif
    
    // Frequency rate
    ros::Rate node_loop_rate(loop_rate);
    
    while (ros::ok()) {
        // Calculate curvature radius      
        curvatureCalculation();

        // Publish curvature calculated
        curvature_pub.publish(curvature_msg);
        
        // If specified, publish calculations for debugging
        #ifdef PLOT_CURVATURE_DATA
            left_pub.publish(left_msg);
            center_pub.publish(center_msg);
            right_pub.publish(right_msg);
        #endif
        
        ros::spinOnce();
        node_loop_rate.sleep(); // sleep the rest of the period
    }
    return 0;
 }

/*
    Callback listening to coeff data of the left lane and storing it
*/
void callbackCoefLeft(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    std::vector<float> temp_data;
    // Extract data to a temporary variable
    for (int i=0; i < msg-> data.size(); i++) {
        temp_data.push_back(msg -> data[i]);
    }
    // Second grade polynomial
    if (temp_data.size() == 3) {
        LeftLane.a = temp_data[2];
        LeftLane.b = temp_data[1];
        LeftLane.c = temp_data[0];
    }
    // First grade polynomial
    if (temp_data.size() == 2) {
        LeftLane.a = 0;
        LeftLane.b = temp_data[1];
        LeftLane.c = temp_data[0];
    }
    // Constant or not detected
    if (temp_data.size() == 1) {
        LeftLane.a = 0;
        LeftLane.b = 0;
        LeftLane.c = temp_data[0];
    }
    ROS_INFO("Left Coef data saved");
}

/*
    Callback listening to coeff data of the center lane and storing it
*/
void callbackCoefCenter(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    std::vector<float> temp_data;
    // Extract data to a temporary variable
    for (int i=0; i < msg-> data.size(); i++) {
        temp_data.push_back(msg -> data[i]);
    }
    // Second grade polynomial
    if (temp_data.size() == 3) {
        CenterLane.a = temp_data[2];
        CenterLane.b = temp_data[1];
        CenterLane.c = temp_data[0];
    }
    // First grade polynomial
    if (temp_data.size() == 2) {
        CenterLane.a = 0;
        CenterLane.b = temp_data[1];
        CenterLane.c = temp_data[0];
    }
    // Constant or not detected
    if (temp_data.size() == 1) {
        CenterLane.a = 0;
        CenterLane.b = 0;
        CenterLane.c = temp_data[0];
    }
    ROS_INFO("Center Coef data saved");
}

/*
    Callback listening to coeff data of the right lane and storing it
*/
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

/*
    Callback listening to degree data of all lanes and storing it
*/
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

/*  Curvature radius is defined by
        R_c = ((1+(df/dx)²)^3/2)/|d²f/dx| 
    
    Second grade curvature radius is then: 
        R_c = 1/(2*a)
        rho = 2*a -> maximum curvature
*/
void curvatureCalculation() {
    
    if(LeftLane.degree > 0) { // Left lane detected
        curvature_msg.left = 2*LeftLane.a;
        #ifdef PLOT_CURVATURE_DATA
            left_msg.data = curvature_msg.left;
        #endif
    }
    else { // Left lane not detected
        //ROS_INFO("Couldn't calculate Left Lane curvature");
        curvature_msg.left = 1000;
        #ifdef PLOT_CURVATURE_DATA
            left_msg.data = 1000;
        #endif
    }

    if(CenterLane.degree > 0) { // Center lane detected
       curvature_msg.center = 2*CenterLane.a;
        #ifdef PLOT_CURVATURE_DATA
            center_msg.data = curvature_msg.center;
        #endif
    }
    else { // Center lane not detected
        curvature_msg.center = 1000;
        #ifdef PLOT_CURVATURE_DATA
            center_msg.data = 1000;
        #endif
    }

    if(RightLane.degree > 0) { // Right lane detected
        curvature_msg.right = 2*RightLane.a;
        #ifdef PLOT_CURVATURE_DATA
            right_msg.data = curvature_msg.right;
        #endif
    }
    else { // Right lane not detected
        curvature_msg.right = 1000; 
        #ifdef PLOT_CURVATURE_DATA
            right_msg.data = 1000;
        #endif
    }
}
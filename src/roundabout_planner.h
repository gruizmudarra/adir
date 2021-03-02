/*
 Node: roundabout_planner 
 Author: German Ruiz Mudarra, UMA Garage Team.
 Created: July 2020
 Last modified: March 2021

 Description: This node rec   
    
 Subscriptions:
    /adir/enable_planning   (adir::enable_t): Enables intersection planners     
    /odom                   (nav_msgs::Odometry): Estimated global position of the car

Publications:
    /tracking_enable        (std_msgs::Bool): Activates or deactivates line_detection_fu steering
    /manual_control/speed   (std_msgs::Int16): Used for giving speed commands to the car
    /adir/environment       (std_msgs::String): Environment where the car is (only used for debugging)
   
 */
#ifndef ROUNDABOUT_PLANNER_H
    #define ROUNDABOUT_PLANNER_H
    #define PRINT_MARKERS

    #include "ros/ros.h"
    #include "math.h"
    #include "iostream"
    using namespace std;


    #include "nav_msgs/Odometry.h"
    #include "std_msgs/Int16.h"
    #include "std_msgs/String.h"
    #include "std_msgs/Bool.h"
    #include "visualization_msgs/Marker.h"
    #include "geometry_msgs/Point.h"

    #include "adir/enable_t.h"
    #include "adir/point2D.h"

    // Message queueing parameters
    static const uint32_t ODOM_QUEUE_SIZE = 1;
    static const uint32_t ENV_QUEUE_SIZE = 1;
    static const uint32_t ENABLE_QUEUE_SIZE = 1;
    static const uint32_t PLANNING_QUEUE_SIZE = 1;
    static const uint32_t MARKERS_QUEUE_SIZE = 20;

    // Cartesian coordinates, this way points can be defined as P(x,y)
    struct position_t {
        double x;
        double y;
        // Constructor for easy mapping
        position_t(double arg_x, double arg_y) : x(arg_x), y(arg_y) {}
    };
    
    // 
    enum maneuver_state_t {IDLE_STATE, DEFINITION_STATE, ENTRY_STATE, CIRCULATION_STATE, EXIT_STATE};

    //
    static const double TRANSIT_RADIUS = 1.6; // Distance between road and center of the roundabout

    // Function promises
    void callbackOdomData(const nav_msgs::Odometry::ConstPtr& msg);
    void callbackADIRData(const adir::enable_t::ConstPtr& msg);
    double bezierLinearScalar(double a, double b, double t);
    position_t bezierLinear(position_t P, position_t Q, double t);
    position_t bezierQuartic(position_t P, position_t Q, position_t R, position_t S, position_t U, double t);
    position_t circunference(position_t P, position_t c, double r, double t);
    double getDistance(position_t p, position_t q);
    position_t getVector(position_t p, position_t q);
    position_t getUnitVector(position_t p, position_t q);
    void selectRestrictionPoints();
    void defineControlPoints();
    void printMarkers();
    void printReference(position_t point);
    void publishReference(position_t r);
    void roundaboutReferenceGenerator();

    // 
    ros::Publisher speed_pub, reference_pub, enable_control_pub;
    std_msgs::Int16 speed_msg;
    adir::point2D reference_msg;
    std_msgs::Bool enable_control_msg;

    #ifdef PRINT_MARKERS
        ros::Publisher marker_pub;
    #endif

    ros::Subscriber odom_sub, adir_sub;
    position_t vehicle_pose(0,0);
    bool enable_roundabout_planner = false;
    int node_entry = 0;
    int node_exit = 0;
    // Variables for ROS parameters
    int loop_rate;
    double lookahead;
    string odometry_topic;
    string planning_topic;
    string reference_topic;
    string planning_markers_topic;
    string control_topic;
    string speed_topic;
    
#endif

/*
 Node: env_class (Environment Classifier)
 Author: German Ruiz Mudarra, May 2020

 Description:
    
 Subscriptions:
 /curvature_calc/all (Information about curvature of the lanes)
 /odom (Global position of the car (not ground-truth))

 Publications:

 */

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/String.h"

static const uint32_t MY_ROS_QUEUE_SIZE = 1;

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
    position_t(double arg_x, double arg_y)
        : x(arg_x), y(arg_y) { }
};

void cb_curvData(const std_msgs::Float32MultiArray::ConstPtr& msg);
void cb_odomData(const nav_msgs::Odometry::ConstPtr& msg);
float get_distance(float x, float y);
std::vector<position_t> define_intersection_nodes();
void environment_classifier_curvature();


curvature_t curvLane;
position_t vehicle_pose(0,0); 

ros::Publisher env_pub;
std_msgs::String env_msg;

 int main (int argc, char **argv) {
    
    ros::init(argc, argv, "env_class");

	ros::NodeHandle nh;

    ros::Subscriber curv_sub = nh.subscribe("/curvature_calc/all", MY_ROS_QUEUE_SIZE, cb_curvData);
    ros::Subscriber odom_sub = nh.subscribe("/odom", MY_ROS_QUEUE_SIZE, cb_odomData);
    env_pub = nh.advertise<std_msgs::String>("/env_class", MY_ROS_QUEUE_SIZE);
    while (ros::ok()) {
        environment_classifier_curvature();
        env_pub.publish(env_msg);
        ros::spinOnce();
        sleep(1);
    }
    return 0;
 }

 void cb_curvData(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    std::vector<float> temp_data;
    for (int i=0; i < msg-> data.size(); i++) {
        temp_data.push_back(msg -> data[i]);
    }
    curvLane.left = temp_data[0];
    curvLane.center = temp_data[1];
    curvLane.right = temp_data[2];
 }

 void cb_odomData(const nav_msgs::Odometry::ConstPtr& msg) {
     vehicle_pose.x = msg -> pose.pose.position.x;
     vehicle_pose.y = msg -> pose.pose.position.y; 
 }

 double get_distance(position_t p1, position_t p2) {
     double dif_x = p1.x - p2.x;
     double dif_y = p1.y - p2.y;
     return sqrt(dif_x*dif_x)+(dif_y*dif_y);
 }

// Define cartesian coordinates of the point to turn
std::vector<position_t> define_intersection_nodes(){
    // Cartesian coordinates matrix of each node
    std::vector<position_t> nodes_matrix;
    //Roundabout 
    //Entries:
    nodes_matrix.push_back(position_t(1.55410,-3.95254));  //1
    nodes_matrix.push_back(position_t(-1.46146,-3.86171)); //2
    nodes_matrix.push_back(position_t(-1.84672,-5.23007)); //3
    nodes_matrix.push_back(position_t(1.75858,-5.81786));  //4
    //Exits
    nodes_matrix.push_back(position_t(-0.674353599548, -3.6364672184));  //5
    nodes_matrix.push_back(position_t(-1.48543524742, -4.52174901962)); //6
    nodes_matrix.push_back(position_t(1.07619309425, -6.2709069252)); //7
    nodes_matrix.push_back(position_t(1.59637641907, -4.8202419281));  //8
    //Curved Crossing 
    //Entries:
    nodes_matrix.push_back(position_t(-3.8876,0.1944)); //9
    nodes_matrix.push_back(position_t(-4.05578,-0.76218)); //10
    nodes_matrix.push_back(position_t(-4.87123,-1.24748)); //11
    //Exits:
    nodes_matrix.push_back(position_t(-3.5204603672, -0.246602073312)); // 15
    nodes_matrix.push_back(position_t(-3.705, -1.6246));    //16
    nodes_matrix.push_back(position_t(-5.28350496292, -1.58464717865)); // 17
    //Regular Crossing:
    //Entries:
    nodes_matrix.push_back(position_t(-5.2882,-4.6787));   //12
    nodes_matrix.push_back(position_t(-4.51369,-4.86417)); //13
    nodes_matrix.push_back(position_t(-4.9159,-5.7948));   //14
    //Exits:
    nodes_matrix.push_back(position_t(-4.81341934204,-4.33024597168)); // 18
    nodes_matrix.push_back(position_t(-3.7626,-5.30747));  //19
    nodes_matrix.push_back(position_t(-5.2837767601,-5.79377508163)); // 20

    return nodes_matrix;
}

void environment_classifier_curvature(){
    string environment = "";
    if (!curvLane.center && !curvLane.right){
        environment = "CRUCE O INTERSECCION";
    }
    else if (abs(curvLane.center) < 300 || abs(curvLane.center) < 300) {
        environment = "CURVA";
        if(curvLane.center < 0 || curvLane.center < 0) {
            environment = environment + " IZQUIERDA";
        }
        else if(curvLane.center > 0 || curvLane.center > 0) {
            environment = environment + " DERECHA";
        }
    }
    else if (abs(curvLane.center) > 300 || abs(curvLane.right) > 300) {
        environment = "RECTA";
    }
    else
    {
        environment = "NO SE PUEDE DISCERNIR A PARTIR DE LA CURVATURA";
    }
    env_msg.data = environment;
}
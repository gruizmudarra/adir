#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt8.h"
#include "geometry_msgs/Point.h"

#include "iostream"

struct car_state {
    double x;
    double y;
    double theta;
    double v;
    
};

static const uint32_t LOOP_RATE = 50; // Hz
using namespace std;

ros::Publisher speed_pub;
std_msgs::Int16 speed_msg;

ros::Publisher steering_pub;
std_msgs::UInt8 steering_msg;

geometry_msgs::Point reference;

void cb_referenceData(const geometry_msgs::Point::ConstPtr& msg);


int main (int argc, char **argv) {
    // Node info
    ros::init(argc, argv, "car_controller");
	ros::NodeHandle nh;
    // Subscriptions
    ros::Subscriber reference_sub = nh.subscribe("/reference", 1000, cb_referenceData);
    // Publications
    speed_pub = nh.advertise<std_msgs::Int16>("/manual_control/speed", 1000);
    steering_pub = nh.advertise<std_msgs::UInt8>("/steering", 1000);
    // Loop rate
    ros::Rate node_loop_rate(LOOP_RATE);

    while (ros::ok()) {
        ros::spinOnce();
        node_loop_rate.sleep();
    }

    return 0;
}

void cb_referenceData(const geometry_msgs::Point::ConstPtr& msg) {
    reference.x = msg -> x;
    reference.y = msg -> y;
    reference.z = msg -> z;
}
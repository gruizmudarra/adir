#include "ros/ros.h"
#include "iostream"
static const uint32_t LOOP_RATE = 50; // Hz
using namespace std;



int main (int argc, char **argv) {
    // Node info
    ros::init(argc, argv, "car_controller");
	ros::NodeHandle nh;
    // Subscriptions

    // Publications

    // Loop rate
    ros::Rate node_loop_rate(LOOP_RATE);

    while (ros::ok()) {
        ros::spinOnce();
        node_loop_rate.sleep();
    }

    return 0;
}
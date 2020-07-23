#include "ros/ros.h"
#include "iostream"
using namespace std;

#include "math.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

// Cartesian coordinates
struct position_t {
    double x;
    double y;
    // Constructor for easy mapping
    position_t(double arg_x, double arg_y) : x(arg_x), y(arg_y) {}
};

struct car_state_t {
    double x;
    double y;
    double theta;
    int theta_deg;
    double v; 
    double speed_state; 
};

struct quaternion_t {
    double x;
    double y;
    double z; 
    double w;
};

void cb_referenceData(const geometry_msgs::Point::ConstPtr& msg);
void cb_odomData(const nav_msgs::Odometry::ConstPtr& msg);
void cb_enable_controlData(const std_msgs::Bool::ConstPtr& msg);
uint8_t saturationU8(int a);
int16_t saturation16(int a);
void orientationControl();
void speedControl();
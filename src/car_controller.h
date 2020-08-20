#ifndef CAR_CONTROLLER_H
#define CAR_CONTROLLER_H
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

#include "adir/point2D.h"

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

static const double ORIENTATION_P = 500.0;

static const double SPEED_P = 10.0;
static const double SPEED_I = 10.0;
static const double SPEED_REFERENCE = 0.1;

void callbackReferenceData(const adir::point2D::ConstPtr& msg);
void callbackOdomData(const nav_msgs::Odometry::ConstPtr& msg);
void callbackEnableControlData(const std_msgs::Bool::ConstPtr& msg);
uint8_t saturationU8(int a);
int16_t saturation16(int a);
void orientationControl();
void speedControl();

int loop_rate;
string odometry_topic;
string reference_topic;
string control_topic;
string speed_topic;
string steering_topic;

static const uint32_t ODOM_QUEUE_SIZE = 1;
static const uint32_t REFERENCE_QUEUE_SIZE = 1;
static const uint32_t CONTROL_QUEUE_SIZE = 1;
static const uint32_t SPEED_QUEUE_SIZE = 1;
static const uint32_t STEERING_QUEUE_SIZE = 1;
#endif
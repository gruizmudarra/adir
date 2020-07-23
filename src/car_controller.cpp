#include "car_controller.h"

static const uint32_t LOOP_RATE = 50; // Hz

ros::Publisher speed_pub;
std_msgs::Int16 speed_msg;

ros::Publisher steering_pub;
std_msgs::UInt8 steering_msg;

geometry_msgs::Point reference;
quaternion_t quat;
double roll, pitch, yaw;

car_state_t car;

static const double ORIENTATION_P = 500.0;

static const double SPEED_P = 10.0;
static const double SPEED_I = 10.0;
static const double SPEED_REFERENCE = 0.1;

bool enable_orientation_control = false;


int main (int argc, char **argv) {
    // Node info
    ros::init(argc, argv, "car_controller");
	ros::NodeHandle nh;
    // Subscriptions
    ros::Subscriber reference_sub = nh.subscribe("/reference", 1000, cb_referenceData);
    ros::Subscriber yaw_sub = nh.subscribe("/odom_ground_truth", 1, cb_odomData);
    ros::Subscriber enable_control_sub = nh.subscribe("/enable_car_control", 1, cb_enable_controlData);
    // Publications
    speed_pub = nh.advertise<std_msgs::Int16>("/manual_control/speed", 1000);
    steering_pub = nh.advertise<std_msgs::UInt8>("/steering", 1000);
    // Loop rate
    ros::Rate node_loop_rate(LOOP_RATE);

    while (ros::ok()) {
        if (enable_orientation_control) {
            orientationControl();
        }
        // speedControl();
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
void cb_odomData(const nav_msgs::Odometry::ConstPtr& msg) {
    car.x = msg -> pose.pose.position.x;
    car.y = msg -> pose.pose.position.y;
    car.v = msg -> twist.twist.linear.x;
    
    quat.x = msg -> pose.pose.orientation.x;
    quat.y = msg -> pose.pose.orientation.y;
    quat.z = msg -> pose.pose.orientation.z;
    quat.w = msg -> pose.pose.orientation.w;
    tf::Quaternion q(quat.x,quat.y,quat.z,quat.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    car.theta = yaw;
    car.theta_deg = car.theta * (180/M_PI);
}

void cb_enable_controlData(const std_msgs::Bool::ConstPtr& msg) {
    enable_orientation_control = msg -> data;
}

uint8_t saturationU8(int a) {
    uint8_t sat;
    if (a > 255) {sat = 255;}
    else if (a < 0) {sat = 0;}
    else {sat = a;}
    return sat;
}

int16_t saturation16(int a) {
    int16_t sat;
    if (a > 32767) {sat = 32767;}
    else if (a < -32768) {sat = -32768;}
    else {sat = a;}
    return sat;
} 

void orientationControl() {  
    position_t incr(reference.x - car.x, reference.y - car.y);
    double phi = atan2(incr.y, incr.x);
    int phi_deg = phi*(180.0/M_PI);
    double err = 0.0;
    char s = '.';
    // Force clockwise turn if theta is in the third quadrant and phi in the second.
  if (car.theta_deg < -90 && phi_deg > 90) {  
    err = 2.0*M_PI + car.theta - phi;
    s = '1';
  }
  // Force counterclockwise turn if phi is in the third quadrant and theta in the second.
  else if (car.theta_deg > 90 && phi_deg < -90) {
    err = -2.0*M_PI + car.theta - phi;
    s = '2';
  }
  else { // First and fourth quadrant angles need no forcing.
    err = car.theta - phi;
    s = '3';
  }
    // w is the output of the controller. 90 is the value of the steering when the car goes straight.
  int w = (ORIENTATION_P * err) + 90;
  steering_msg.data = saturationU8(w);
  steering_pub.publish(steering_msg);
  // ROS_INFO("[DEBUG]orientationControl -> CASE = %c, THETA=%d, PHI=%d, ERROR=%.3f w=%d, STEER=%d", s, car.theta_deg, phi_deg, err, w, steering_msg.data);
}

void speedControl() {
    double err = SPEED_REFERENCE - car.v;
    car.speed_state += SPEED_I*err;
    speed_msg.data = saturation16(car.speed_state);
    if (abs(err) > 0.01) {
        speed_pub.publish(speed_msg);
        ROS_INFO("[DEBUG]speedControl -> Speed_r = %.3f, V_odom = %.3f, ERROR=%.3f, SPEED= %d", SPEED_REFERENCE, car.v, err, speed_msg.data);
    }
}
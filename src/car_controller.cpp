#include "car_controller.h"

static const uint32_t LOOP_RATE = 50; // Hz
/*
ros::Publisher speed_pub;
std_msgs::Int16 speed_msg;
*/
ros::Publisher steering_pub;
std_msgs::UInt8 steering_msg;

geometry_msgs::Point reference;
quaternion_t quat;
double roll, pitch, yaw;

car_state_t car;

static const double ORIENTATION_P = 500.0;

/*static const double SPEED_P = 1.0;
static const double SPEED_REFERENCE = 1.66;
*/
bool enable_control = false;


int main (int argc, char **argv) {
    // Node info
    ros::init(argc, argv, "car_controller");
	ros::NodeHandle nh;
    // Subscriptions
    ros::Subscriber reference_sub = nh.subscribe("/reference", 1000, cb_referenceData);
    ros::Subscriber yaw_sub = nh.subscribe("/odom_ground_truth", 1, cb_odomData);
    ros::Subscriber enable_control_sub = nh.subscribe("/enable_car_control", 1, cb_enable_controlData);
    // Publications
    //speed_pub = nh.advertise<std_msgs::Int16>("/manual_control/speed", 1000);
    steering_pub = nh.advertise<std_msgs::UInt8>("/steering", 1000);
    // Loop rate
    ros::Rate node_loop_rate(LOOP_RATE);

    while (ros::ok()) {
        if (enable_control) {
            // car_control();
            computeControlCmd();
            steering_pub.publish(steering_msg);
        }
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
}

void cb_enable_controlData(const std_msgs::Bool::ConstPtr& msg) {
    enable_control = msg -> data;
}

double angle_wrap(double a) {
    double angle;
    if (a > M_PI) {angle = a - 2*M_PI;}
    else if (a < -1*M_PI) {angle = a + 2*M_PI;}
    else {angle = a;}
    return angle;
}

void car_control() {
    position_t incr(reference.x - car.x, reference.y - car.y);
    double phi = atan2(incr.y, incr.x);
    phi = angle_wrap(phi);
    double orientation_error = phi - car.theta;
    //double speed_error = SPEED_REFERENCE - car.v;
    
    // Proportional control for speed
    //speed_msg.data = (int16_t)max(0.0,SPEED_P*speed_error);

    // Proportional control for orientation
    double w = ORIENTATION_P*orientation_error + 90.0;

    uint8_t steering = (min((int)max(0.0,w),255));
    steering_msg.data = steering;
    ROS_INFO("[DEBUG]car_control -> THETA=%.3f, PHI=%.3f, ERROR=%.3f w=%f, STEER=%d",car.theta, phi, orientation_error, w, steering_msg.data);
}   

void computeControlCmd() {  
    position_t incr(reference.x - car.x, reference.y - car.y);
    double phi = atan2(incr.y, incr.x);
    double err = 0.0;
  if (car.theta < -M_PI/2.0 && phi > M_PI/2.0) {
    err = -2.0*M_PI + car.theta - phi;
  }
  else if (car.theta > M_PI/2.0 && phi < -M_PI/2.0) {
    err = 2.0*M_PI + car.theta - phi;
  }
  else {
    err = car.theta - phi;
  }
    
  int w = (int)(ORIENTATION_P * err) + 90;
  steering_msg.data = (uint8_t)w;
  ROS_INFO("[DEBUG]computeControlCmd -> THETA=%.3f, PHI=%.3f, ERROR=%.3f w=%d, STEER=%d",car.theta, phi, err, w, steering_msg.data);
}
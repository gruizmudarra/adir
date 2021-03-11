#include "car_controller.h"

int main (int argc, char **argv) {
    // Node info
    ros::init(argc, argv, "car_controller");
	ros::NodeHandle nh;
    
    // Parameters
    nh.param<int>("/loop_rate", loop_rate,50);
    nh.param<string>("/odom_topic", odometry_topic, "/odom");
    nh.param<string>("/reference_topic", reference_topic, "/adir/reference");
    nh.param<string>("/control_topic", control_topic, "/adir/enable_control");
    nh.param<string>("/speed_topic", speed_topic, "/manual_control/speed");
    nh.param<string>("/steering_topic", steering_topic, "/steering");
        
    // Subscriptions
    reference_sub = nh.subscribe(reference_topic, REFERENCE_QUEUE_SIZE, callbackReferenceData);
    odom_sub = nh.subscribe(odometry_topic, ODOM_QUEUE_SIZE, callbackOdomData);
    control_sub = nh.subscribe(control_topic, CONTROL_QUEUE_SIZE, callbackEnableControlData);
    
    // Publications
    //speed_pub = nh.advertise<std_msgs::Int16>(speed_topic, SPEED_QUEUE_SIZE);
    steering_pub = nh.advertise<std_msgs::UInt8>(steering_topic, STEERING_QUEUE_SIZE);
    
    // Frequency rate
    ros::Rate node_loop_rate(loop_rate);

    // Main loop
    while (ros::ok()) {
        if (enable_orientation_control) { // If intersection planner enables control
            orientationControl();
        }
        // speedControl();
        ros::spinOnce();
        node_loop_rate.sleep();
    }
    return 0;
}

// Callback listening to the reference point
void callbackReferenceData(const adir::point2D::ConstPtr& msg) {
    reference.x = msg -> x;
    reference.y = msg -> y;
}

// Callback listening to the position of the car
void callbackOdomData(const nav_msgs::Odometry::ConstPtr& msg) {
    car.x = msg -> pose.pose.position.x;
    car.y = msg -> pose.pose.position.y;
    car.v = msg -> twist.twist.linear.x;
    
    tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    car.theta = yaw;
    car.theta_deg = car.theta * (180/M_PI);
}

// Callback listening to the enable message
void callbackEnableControlData(const std_msgs::Bool::ConstPtr& msg) {
    enable_orientation_control = msg -> data;
}

// This function is used to avoid overflow in 8 bit values for the steering
uint8_t saturationU8(int a) {
    uint8_t sat;
    if (a > 255) {sat = 255;}
    else if (a < 0) {sat = 0;}
    else {sat = a;}
    return sat;
}

/*
int16_t saturation16(int a) {
    int16_t sat;
    if (a > 32767) {sat = 32767;}
    else if (a < -32768) {sat = -32768;}
    else {sat = a;}
    return sat;
}
*/


void orientationControl() {  
    position_t incr(reference.x - car.x, reference.y - car.y); // Vector from the car to the reference
    double phi = atan2(incr.y, incr.x); // Angle of the vector
    int phi_deg = phi*(180.0/M_PI); // Angle of the vector in degrees
    double err = 0.0;
    char s = '.';

    // Force clockwise turn if theta is in the third quadrant and phi in the second.
  if (car.theta_deg < -90 && phi_deg > 90) {  
    err = 2.0*M_PI + car.theta - phi;
    s = '1';
  }
  // Force counterclockwise turn if phi is in the third quadrant and theta in the second
  else if (car.theta_deg > 90 && phi_deg < -90) {
    err = -2.0*M_PI + car.theta - phi;
    s = '2';
  }
  else { // First and fourth quadrant angles need no forcing
    err = car.theta - phi;
    s = '3';
  }
    // w is the output of the controller. 90 is the value of the steering when the car goes straight.
  int w = (ORIENTATION_P * err) + 90;
  steering_msg.data = saturationU8(w);
  steering_pub.publish(steering_msg);
  // ROS_INFO("[DEBUG]orientationControl -> CASE = %c, THETA=%d, PHI=%d, ERROR=%.3f w=%d, STEER=%d", s, car.theta_deg, phi_deg, err, w, steering_msg.data);
}

/*
void speedControl() {
    double err = SPEED_REFERENCE - car.v;
    car.speed_state += SPEED_I*err;
    speed_msg.data = saturation16(car.speed_state);
    if (abs(err) > 0.01) {
        speed_pub.publish(speed_msg);
        ROS_INFO("[DEBUG]speedControl -> Speed_r = %.3f, V_odom = %.3f, ERROR=%.3f, SPEED= %d", SPEED_REFERENCE, car.v, err, speed_msg.data);
    }
}
*/
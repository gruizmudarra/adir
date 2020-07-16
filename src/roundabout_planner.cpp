#include "roundabout_planner.h"

static const uint32_t LOOP_RATE = 10; // Hz
static const uint32_t ODOM_QUEUE_SIZE = 1;
static const uint32_t ENV_QUEUE_SIZE = 1;
static const uint32_t ENABLE_QUEUE_SIZE = 1;

static const position_t R0(0,0); // Roundabout center
static const double TRANSIT_RADIUS; // Distance between road and center of the roundabout

position_t vehicle_pose(0,0);
string environment = "";
bool enable_roundabout_planner = false;

int main(int argc, char **argv) {
    // Node info
    ros::init(argc, argv, "roundabout_planner");
	ros::NodeHandle nh;
    
    // Subscriptions
    ros::Subscriber odom_sub = nh.subscribe("/odom_ground_truth", ODOM_QUEUE_SIZE, cb_odomData);
    ros::Subscriber env_sub = nh.subscribe("/env_class", ENV_QUEUE_SIZE, cb_envData);
    ros::Subscriber enable_sub = nh.subscribe("/enable_roundabout_planner", ENABLE_QUEUE_SIZE, cb_enableData);
    // Publications

    // Loop rate
    ros::Rate node_loop_rate(LOOP_RATE);

    while (ros::ok())
    {
        if (enable_roundabout_planner) {
            roundabout_reference_generator();
        }
        ros::spinOnce();
        node_loop_rate.sleep();
    }
    return 0;
}

 void cb_odomData(const nav_msgs::Odometry::ConstPtr& msg) {
     vehicle_pose.x = msg -> pose.pose.position.x;
     vehicle_pose.y = msg -> pose.pose.position.y; 
 }

void cb_envData(const std_msgs::String::ConstPtr& msg) {
    environment = msg -> data;
}

void cb_enableData(const std_msgs::Bool::ConstPtr& msg) {
    enable_roundabout_planner = msg -> data;
}

double bezier_linear_scalar(double p0, double p1, double t) {
    return p0 + ((p1-p0)*t);
}

position_t bezier_linear(position_t p1, position_t p2, double t) {
    position_t p(0,0);
    p.x = bezier_linear_scalar(p1.x,p2.x,t);
    p.y = bezier_linear_scalar(p1.y,p2.y,t);
    return p;
}

// Evaluates the Bezier curve for a value of t using De Casteljau’s algorithm. Easier to see graphically.
// I consulted the time used in this algorithm compared to calculate the value using the equation directly, 
// which resulted in the algorithm being 5x faster.
position_t bezier_quartic(position_t p1, position_t p2, position_t p3, position_t p4, position_t p5, double t) {
    // First grade interpolation
    position_t pa = bezier_linear(p1,p2,t);
    position_t pb = bezier_linear(p2,p3,t);
    position_t pc = bezier_linear(p3,p4,t);
    position_t pd = bezier_linear(p4,p5,t);
    // Second grade interpolation
    position_t pm = bezier_linear(pa,pb,t);
    position_t pn = bezier_linear(pb,pc,t);
    position_t pl = bezier_linear(pc,pd,t);
    // Third grade interpolation
    position_t pu = bezier_linear(pm,pn,t);
    position_t pv = bezier_linear(pn,pl,t);
    // Fourth grade interpolation
    position_t pf = bezier_linear(pu,pv,t);
    
    return pf;    
}

 double get_distance(position_t p, position_t q) {
     double dif_x = q.x - p.x;
     double dif_y = q.y - p.y;
     return sqrt(pow(dif_x, 2.0)+pow(dif_y,2.0));
 }

 position_t get_vector(position_t p, position_t q) {
     double dif_x = q.x - p.x;
     double dif_y = q.y - p.y;
     position_t pq(dif_x, dif_y);
    return pq;
 }

 position_t get_unit_vector(position_t p, position_t q) {
     position_t pq = get_vector(p,q);
     double dist = get_distance(p,q);
     position_t pq_unit(0,0);
     if(dist) {
        pq_unit.x = pq.x/dist;
        pq_unit.y = pq.y/dist;
     }
     return pq_unit;
 }

 void select_restriction_points(position_t& ientry1, position_t& ientry2, position_t& iexit1, position_t& iexit2, position_t& rexit) {
     // Choose geometric points of the roundabout depending on the entry/exit
    int entry = environment.back();
    int exit;
    cout << "Insert exit node (5-8): \n" ;
    cin >> exit;
    switch(entry){
        case '1': 
            //I1 e I2
            break;
        case '2':
            //I1 e I2 
            break;
        case '3': 
            //I1 e I2
            break;
        case '4': 
            //I1 e I2
            break;
        default: 
            cout << "Exception on entry node. \n";
            break;
    }
    switch(exit){
        case '5': 
            //R2, I3 e I4
            break;
        case '6':
            //R2, I3 e I4 
            break;
        case '7': 
            //R2, I3 e I4
            break;
        case '8': 
            //R2, I3 e I4
            break;
        default: 
            cout << "Exception on exit node. \n";
            break;
    }
 }

void roundabout_reference_generator() {
    position_t r1 = vehicle_pose;
    double l0 = get_distance(r1,R0);
    cout << "l0 = " << l0 << "\n";
    position_t r2(0,0);
    position_t i1(0,0), i2(0,0), i3(0,0), i4(0,0);

    select_restriction_points(i1,i2,i3,i4,r2);
    // Entry control points
    position_t p0(0,0), p1(0,0), p2(0,0), p3(0,0), p4(0,0);
    
    p0.x = R0.x + l0*get_unit_vector(r1,R0).x;
    p0.y = R0.y + l0*get_unit_vector(r1,R0).y;

    double l1 = 0.75*l0;
    p1.x = R0.x + l1*get_unit_vector(r1,R0).x;
    p1.y = R0.y + l1*get_unit_vector(r1,R0).y;

    double l2;
    p2.x = i1.x + l2*get_unit_vector(i1,i2).x;
    p2.y = i1.y + l2*get_unit_vector(i1,i2).y;
    
    double l4;
    double phi_entry = l4/TRANSIT_RADIUS;
    p4.x = TRANSIT_RADIUS*cos(atan2(r1.y-R0.y,r1.x-R0.x)+phi_entry)+R0.x;
    p4.y = TRANSIT_RADIUS*sin(atan2(r1.y-R0.y,r1.x-R0.x)+phi_entry)+R0.y;

    position_t normal_entry = get_unit_vector(p4,R0);
    position_t tangent_entry(normal_entry.y, -1*normal_entry.x); 
    double l3;
    p3.x = p4.x + l3*tangent_entry.x;
    p3.y = p4.y + l3*tangent_entry.y;

    // Exit control points (simetric to the entry calculation)
    position_t p5(0,0), p6(0,0), p7(0,0), p8(0,0), p9(0,0);
    
    double l5;
    double phi_exit = l5/TRANSIT_RADIUS;
    p5.x = TRANSIT_RADIUS*cos(atan2(r2.y-R0.y,r2.x-R0.x)-phi_exit)+R0.x;
    p5.y = TRANSIT_RADIUS*sin(atan2(r2.y-R0.y,r2.x-R0.x)-phi_exit)+R0.y;

    position_t normal_exit = get_unit_vector(p5,R0);
    position_t tangent_exit(normal_exit.y, -1*normal_exit.x); 
    double l6;
    p6.x = p5.x + l6*tangent_exit.x;
    p6.y = p5.y + l6*tangent_exit.y;

    double l7;
    p7.x = i3.x + l7*get_unit_vector(i3,i4).x;
    p7.y = i3.y + l7*get_unit_vector(i3,i4).y;

    double l9 = get_distance(r2,R0);
    p9.x = R0.x + l9*get_unit_vector(r2,R0).x;
    p9.y = R0.y + l9*get_unit_vector(r2,R0).y;

    double l8 = 0.75*l9;
    p8.x = R0.x + l8*get_unit_vector(r2,R0).x;
    p8.y = R0.y + l8*get_unit_vector(r2,R0).y;

    
}
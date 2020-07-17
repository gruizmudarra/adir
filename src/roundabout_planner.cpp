#include "roundabout_planner.h"

static const uint32_t LOOP_RATE = 50; // Hz
static const uint32_t ODOM_QUEUE_SIZE = 1;
static const uint32_t ENV_QUEUE_SIZE = 1;
static const uint32_t ENABLE_QUEUE_SIZE = 1;

position_t vehicle_pose(0,0);
string environment = "";
bool enable_roundabout_planner = false;
int node = 0;
double t = 0;
maneuver_state_t maneuver_state = IDLE;


static const position_t R0(0,-5.1); // Roundabout center
static const double TRANSIT_RADIUS = 1.6; // Distance between road and center of the roundabout
position_t r1(0,0), r2(0,0), i1(0,0), i2(0,0), i3(0,0), i4(0,0);
position_t p0(0,0), p1(0,0), p2(0,0), p3(0,0), p4(0,0), p5(0,0), p6(0,0), p7(0,0), p8(0,0), p9(0,0);
bool control_points_defined = false;

static const double LOOKAHEAD =  0.1;

#ifdef PRINT_MARKERS
bool markers_printed = false;
ros::Publisher marker_pub;
#endif

ros::Publisher speed_pub;
std_msgs::Int16 speed_msg;

int main(int argc, char **argv) {
    // Node info
    ros::init(argc, argv, "roundabout_planner");
	ros::NodeHandle nh;
    
    // Subscriptions
    ros::Subscriber odom_sub = nh.subscribe("/odom_ground_truth", ODOM_QUEUE_SIZE, cb_odomData);
    ros::Subscriber env_sub = nh.subscribe("/env_class", ENV_QUEUE_SIZE, cb_envData);
    ros::Subscriber node_sub = nh.subscribe("/env_node", ENV_QUEUE_SIZE, cb_nodeData);
    ros::Subscriber enable_sub = nh.subscribe("/enable_roundabout_planner", ENABLE_QUEUE_SIZE, cb_enableData);
    // Publications
    #ifdef PRINT_MARKERS
    marker_pub = nh.advertise<visualization_msgs::Marker>("/control_points_marker", 20);
    #endif
    
    speed_pub = nh.advertise<std_msgs::Int16>("/manual_control/speed", 1000);
    
    // Loop rate
    ros::Rate node_loop_rate(LOOP_RATE);

    while (ros::ok()) {
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

void cb_nodeData(const std_msgs::Int16::ConstPtr& msg) {
    node = msg -> data;
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

position_t circunference(position_t p, position_t c, double r, double t) {
    double phi1 = atan2(p.y-c.y,p.x-c.x);
    position_t p(0,0);
    p.x = c.x + r*cos(phi1+t);
    p.y = c.y + r*sin(phi1+t);
    return p;
}
 double get_distance(position_t p, position_t q) {
     double dif_x = q.x - p.x;
     double dif_y = q.y - p.y;
     return sqrt(dif_x*dif_x+dif_y*dif_x);
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

 void select_restriction_points() {
     // Choose geometric points of the roundabout depending on the entry/exit
    cout << "You are right now in the entry node " << node << ".\n";
    int entry = node;
    int exit;
    cout << "Insert exit node (5-8): \n" ;
    cin >> exit;
    switch(entry){
        case 1: 
            //I1 e I2
            i1.x = 1.5932;
            i1.y = -4.2102;

            i2.x = 1.3563;
            i2.y = -3.8594;
            break;
        case 2:
            //I1 e I2 
            i1.x = -1.2862;
            i1.y = -3.8050;
            
            i2.x = -1.5516;
            i2.y = -4.1224;
            break;
        case 3: 
            //I1 e I2
            i1.x = -1.8500;
            i1.y = -5.0816;

            i2.x = -1.7652;
            i2.y = -5.5090;
            break;
        case 4: 
            //I1 e I2
            i1.x = 1.5921;
            i1.y = -5.9507;

            i2.x = 1.7551 ;
            i2.y = -5.5638;
            break;
        default: 
            cout << "Exception on entry node. \n";
            break;
    }
    cout << "Restriction points of the entry created. \n";
    switch(exit){
        case 5: 
            //R2, I3 e I4
            i3.x = 1.5932;
            i3.y = -4.2102;

            i4.x = 1.7583 ;
            i4.y = -4.6069;

            r2.x = 2.3609;
            r2.y = -4.0260;
            break;
        case 6:
            //R2, I3 e I4
            i3.x =-1.2862;
            i3.y = -3.8050;

            i4.x = -0.9478;
            i4.y = -3.5434;

            r2.x = -1.5341;
            r2.y = -3.2025; 
            break;
        case 7: 
            //R2, I3 e I4
            i3.x = -1.8500;
            i3.y = -5.0816;

            i4.x = -1.7707;
            i4.y = -4.6651;

            r2.x = -2.3795;
            r2.y = -4.8837;
            break;
        case 8: 
            //R2, I3 e I4
            i3.x = 1.5921;
            i3.y = -5.9507;

            i4.x = 1.3464;
            i4.y = -6.2988;

            r2.x = 2.0279;
            r2.y = -6.4677;
            break;
        default: 
            cout << "Exception on exit node. \n";
            break;
    }
    cout << "Restriction points of the exit created. \n";
 }


 void define_control_points() {
     // Entry control points
    cout << "Defining control points. \n";
    r1 = vehicle_pose;
    double l0 = get_distance(R0, r1);
    cout << "l0 = " << l0 << "\n";
    p0.x = R0.x + l0*get_unit_vector(R0,r1).x;
    p0.y = R0.y + l0*get_unit_vector(R0,r1).y;

    double l1 = 0.85*l0;
    p1.x = R0.x + l1*get_unit_vector(R0,r1).x;
    p1.y = R0.y + l1*get_unit_vector(R0,r1).y;

    double l2 =0.15;
    p2.x = i1.x + l2*get_unit_vector(i2,i1).x;
    p2.y = i1.y + l2*get_unit_vector(i2,i1).y;
    
    double l4 = 0.5;
    double phi_entry = l4/TRANSIT_RADIUS;
    p4.x = TRANSIT_RADIUS*cos(atan2(r1.y-R0.y,r1.x-R0.x)+phi_entry)+R0.x;
    p4.y = TRANSIT_RADIUS*sin(atan2(r1.y-R0.y,r1.x-R0.x)+phi_entry)+R0.y;

    position_t normal_entry = get_unit_vector(p4,R0);
    position_t tangent_entry(normal_entry.y, -1*normal_entry.x); 
    double l3 = 0.2;
    p3.x = p4.x + l3*tangent_entry.x;
    p3.y = p4.y + l3*tangent_entry.y;

    // Exit control points (simetric to the entry calculation)
    double l5 = l4;
    double phi_exit = l5/TRANSIT_RADIUS;
    p5.x = TRANSIT_RADIUS*cos(atan2(r2.y-R0.y,r2.x-R0.x)-phi_exit)+R0.x;
    p5.y = TRANSIT_RADIUS*sin(atan2(r2.y-R0.y,r2.x-R0.x)-phi_exit)+R0.y;

    position_t normal_exit = get_unit_vector(p5,R0);
    position_t tangent_exit(normal_exit.y, -1*normal_exit.x); 
    double l6 = l3;
    p6.x = p5.x + l6*tangent_exit.x;
    p6.y = p5.y + l6*tangent_exit.y;

    double l7 = l2;
    p7.x = i3.x + l7*get_unit_vector(i4,i3).x;
    p7.y = i3.y + l7*get_unit_vector(i4,i3).y;

    double l9 = get_distance(R0,r2);
    p9.x = R0.x + l9*get_unit_vector(R0,r2).x;
    p9.y = R0.y + l9*get_unit_vector(R0,r2).y;

    double l8 = 0.85*l9;
    p8.x = R0.x + l8*get_unit_vector(R0,r2).x;
    p8.y = R0.y + l8*get_unit_vector(R0,r2).y;
 }

#ifdef PRINT_MARKERS
void print_markers() {
    // Restriction points configuration
    visualization_msgs::Marker restriction_points;
    restriction_points.header.frame_id = "world";
    restriction_points.header.stamp = ros::Time::now();
    restriction_points.ns = "restriction_points";
    restriction_points.action = visualization_msgs::Marker::ADD;
    restriction_points.pose.orientation.w  = 1.0;
    
    restriction_points.id = 0;
    
    restriction_points.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    restriction_points.scale.x = 0.05;
    restriction_points.scale.y = 0.05;

    /*// Points are red
    restriction_points.color.r = 1.0;
    restriction_points.color.a = 1.0;
*/
    geometry_msgs::Point p;

    position_t restriction_points_vector[] = {R0, r1, r2, i1, i2, i3, i4};
    
    for(int k = 0; k <= 6; k++) {
        std_msgs::ColorRGBA c;
        if (k < 3) {
            c.r = 1.0;
        }
        else if (k >=3 && k<5) {
            c.r = 1;
            c.g = 1;
        }
        else {
            c.r = 1.0;
            c.g = 0.5;
        }
        c.a = 1.0;

        p.x = restriction_points_vector[k].x;
        p.y = restriction_points_vector[k].y;
        p.z = 0.25;
        restriction_points.points.push_back(p);
        restriction_points.colors.push_back(c);
        
    }
    // Control points configuration
    visualization_msgs::Marker control_points;
    control_points.header.frame_id = "world";
    control_points.header.stamp = ros::Time::now();
    control_points.ns = "control_points";
    control_points.action = visualization_msgs::Marker::ADD;
    control_points.pose.orientation.w  = 1.0;
    
    control_points.id = 0;
    
    control_points.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    control_points.scale.x = 0.05;
    control_points.scale.y = 0.05;
    /*
    // Points are green
    control_points.color.b = 1.0;
    control_points.color.a = 1.0;
*/
    position_t control_points_vector[] = {p0, p1, p2, p3, p4, p5, p6, p7, p8, p9};
    geometry_msgs::Point q;
    int ii = 0.1;
    for(int kk = 0; kk <= 9; kk++) {
        std_msgs::ColorRGBA c;
        switch(kk) {
            case 0: 
                c.g = 0.2;
                break;
            case 1: 
                c.g = 0.4;
                break;
            case 2: 
                c.g = 0.6;
                break;
            case 3: 
                c.g = 0.8;
                break;
            case 4: 
                c.g = 1.0;
                break;
            case 5: 
                c.b = 0.2;
                break;
            case 6: 
                c.b = 0.4;
                break;
            case 7: 
                c.b = 0.6;
                break;
            case 8: 
                c.b = 0.8;
                break;
            case 9: 
                c.b = 1.0;
                break;
        }
        c.a = 1.0;
        q.x = control_points_vector[kk].x;
        q.y = control_points_vector[kk].y;
        q.z = 0.25;
        control_points.points.push_back(q);
        control_points.colors.push_back(c);
        ii = ii + 0.1;
    }
    marker_pub.publish(restriction_points);
    marker_pub.publish(control_points);
}
#endif

void roundabout_reference_generator() {
    if (!control_points_defined) {
        speed_msg.data = 0;
        speed_pub.publish(speed_msg);
        cout << "Roundabout detected! \n";
        select_restriction_points();
        define_control_points();
        #ifdef PRINT_MARKERS
        print_markers();
        #endif
        control_points_defined = true;
        maneuver_state = ENTRY_STATE;
    }
    position_t reference(0,0);
    
    switch (maneuver_state) {
        case ENTRY_STATE:
            reference = bezier_quartic(p0,p1,p2,p3,p4,t);
            if (get_distance(vehicle_pose, reference) < LOOKAHEAD) {
                t+= 0.1;
            }
            if(get_distance(vehicle_pose,p4) < LOOKAHEAD) {
                t = 0;
                maneuver_state = CIRCULATION_STATE;
            }
            break;
        case CIRCULATION_STATE:
            reference = circunference(p4,R0,TRANSIT_RADIUS,t);
            if (get_distance(vehicle_pose, reference) < LOOKAHEAD) {
                t+= 0.1;
            }
            if(get_distance(vehicle_pose,p5) < LOOKAHEAD) {
                t = 0;
                maneuver_state = EXIT_STATE;
            }
            break;
        case EXIT_STATE:
            reference = bezier_quartic(p5,p6,p7,p8,p9,t);
            if (get_distance(vehicle_pose, reference) < LOOKAHEAD) {
                t+= 0.1;
            }
            if(get_distance(vehicle_pose,p9) < LOOKAHEAD) {
                t = 0;
                maneuver_state = IDLE;
            }
            break;
        case IDLE:
            enable_roundabout_planner = false;
            control_points_defined = false;
            p0 = position_t(0,0);
            p1 = position_t(0,0);
            p2 = position_t(0,0);
            p3 = position_t(0,0);
            p4 = position_t(0,0);
            p5 = position_t(0,0);
            p6 = position_t(0,0);
            p7 = position_t(0,0);
            p8 = position_t(0,0);
            p9 = position_t(0,0);
        break;
    }
}

#include "roundabout_planner.h"

position_t vehicle_pose(0,0);
position_t reference(0,0);
bool enable_roundabout_planner = false;
int node_entry = 0;
int node_exit = 0;
maneuver_state_t maneuver_state = DEFINITION_STATE;
double t = 0.1;
position_t r0(0,-5.1), r1(0,0), r2(0,0), i1(0,0), i2(0,0), i3(0,0), i4(0,0);
position_t p0(0,0), p1(0,0), p2(0,0), p3(0,0), p4(0,0), p5(0,0), p6(0,0), p7(0,0), p8(0,0), p9(0,0);

#ifdef PRINT_MARKERS
    ros::Publisher marker_pub;
#endif

ros::Publisher speed_pub;
std_msgs::Int16 speed_msg;

ros::Publisher reference_pub;
adir::point2D reference_msg;

ros::Publisher enable_control_pub;
std_msgs::Bool enable_control_msg;

int main(int argc, char **argv) {
    // Node info
    ros::init(argc, argv, "roundabout_planner");
	ros::NodeHandle nh;
    
    // Parameters
    nh.param<int>("/loop_rate", loop_rate, 50);
    nh.param<double>("/lookahead", lookahead, 0.01);
    nh.param<string>("/odom_topic", odometry_topic, "/odom");
    nh.param<string>("/planning_topic", planning_topic, "/adir/enable_planning");
    nh.param<string>("/planning_markers_topic", planning_markers_topic, "/adir/planning_markers");
    nh.param<string>("/reference_topic", reference_topic, "/adir/reference");
    nh.param<string>("/control_topic", control_topic, "/adir/enable_control");
    nh.param<string>("/speed_topic", speed_topic, "/manual_control/speed");
    
    
    // Subscriptions
    ros::Subscriber odom_sub = nh.subscribe(odometry_topic, ODOM_QUEUE_SIZE, callbackOdomData);
    ros::Subscriber adir_sub = nh.subscribe(planning_topic, PLANNING_QUEUE_SIZE, callbackADIRData);
    // Publications
    #ifdef PRINT_MARKERS
    marker_pub = nh.advertise<visualization_msgs::Marker>(planning_markers_topic, MARKERS_QUEUE_SIZE);
    #endif
    reference_pub = nh.advertise<adir::point2D>(reference_topic,1000);
    enable_control_pub = nh.advertise<std_msgs::Bool>(control_topic, 1000);
    speed_pub = nh.advertise<std_msgs::Int16>(speed_topic, 1000);
    
    // Loop rate
    ros::Rate node_loop_rate(loop_rate);

    while (ros::ok()) {
        if (enable_roundabout_planner) {
            roundaboutReferenceGenerator();
        }
        ros::spinOnce();
        node_loop_rate.sleep();
    }
    return 0;
}

 void callbackOdomData(const nav_msgs::Odometry::ConstPtr& msg) {
     vehicle_pose = position_t(msg -> pose.pose.position.x, msg -> pose.pose.position.y);
 }

void callbackADIRData(const adir::enable_t::ConstPtr& msg) {
    enable_roundabout_planner = msg -> roundabout;
    node_entry = msg -> node_entry;
    node_exit = msg -> node_exit;
}

double bezierLinearScalar(double a, double b, double t) {
    return a + ((b-a)*t);
}

position_t bezierLinear(position_t P, position_t Q, double t) {
    position_t R(0,0);
    R.x = bezierLinearScalar(P.x,Q.x,t);
    R.y = bezierLinearScalar(P.y,Q.y,t);
    return R;
}

// Evaluates the Bezier curve for a value of t using De Casteljau’s algorithm. Easier to see graphically.
// I consulted the time used in this algorithm compared to calculate the value using the equation directly, 
// which resulted in the algorithm being 5x faster.
position_t bezierQuartic(position_t P, position_t Q, position_t R, position_t S, position_t U, double t) {
    // First grade interpolation
    position_t pa = bezierLinear(P,Q,t);
    position_t pb = bezierLinear(Q,R,t);
    position_t pc = bezierLinear(R,S,t);
    position_t pd = bezierLinear(S,U,t);
    // Second grade interpolation
    position_t pm = bezierLinear(pa,pb,t);
    position_t pn = bezierLinear(pb,pc,t);
    position_t pl = bezierLinear(pc,pd,t);
    // Third grade interpolation
    position_t pu = bezierLinear(pm,pn,t);
    position_t pv = bezierLinear(pn,pl,t);
    // Fourth grade interpolation
    position_t pf = bezierLinear(pu,pv,t);
    return pf;    
}

position_t circunference(position_t P, position_t c, double r, double t) {
    double phi = atan2(P.y - c.y, P.x - c.x);
    position_t Q(c.x + r*cos(phi+t), c.y + r*sin(phi+t));
    return Q;
}

 double getDistance(position_t p, position_t q) {
     return sqrt((q.x - p.x)*(q.x - p.x)+(q.y - p.y)*(q.y - p.y));
 }

 position_t getVector(position_t p, position_t q) {
     position_t pq(q.x - p.x, q.y - p.y);
    return pq;
 }

position_t getUnitVector(position_t p, position_t q) {
     position_t pq = getVector(p,q);
     double dist = getDistance(p,q);
     position_t pq_unit(0,0);
     if(dist > 0.001) {
        pq_unit = position_t(pq.x/dist, pq.y/dist);
     }
     return pq_unit;
 }


 void selectRestrictionPoints() {
     // Choose geometric points of the roundabout depending on the entry/exit
    r1 = vehicle_pose;
    switch(node_entry){
        case 1: 
            i1 = position_t(1.5932,-4.2102);
            i2 = position_t(1.3563,-3.8505);
            break;
        case 2:
            i1 = position_t(-1.2862,-3.8050);
            i2 = position_t(-1.5516,-4.1224);
            break;
        case 3: 
            i1 = position_t(-1.8500,-5.0816);
            i2 = position_t(-1.7652,-5.5090);
            break;
        case 4: 
            i1 = position_t(1.5921,-5.9507);
            i2 = position_t(1.7551,-5.5638);
            break;
        default: 
            cout << "Exception on entry node. \n";
            break;
    }
    cout << "Restriction points of the entry created. \n";
    switch(node_exit){
        case 5:
            i3 = position_t(-1.2862,-3.8050);
            i4 = position_t(-0.9478,-3.5434);
            r2 = position_t(-1.9152,-2.8758);
            break;
        case 6: 
            i3 = position_t(-1.8500,-5.0816);
            i4 = position_t(-1.7707,-4.6651);
            r2 = position_t(-2.9942,-4.8727);
            break;
        case 7: 
            i3 = position_t(1.5921,-5.9507);
            i4 = position_t(1.3464,-6.2988);
            r2 = position_t(2.42829,-6.6881);
            break;
        case 8: 
            i3 = position_t(1.5932,-4.2102);
            i4 = position_t(1.7583,-4.6069);
            r2 = position_t(2.6465,-3.8140);
            break;
        default: 
            cout << "Exception on exit node. \n";
            break;
    }
    cout << "Restriction points of the exit created. \n";
 }


 void defineControlPoints() {
    cout << "Defining control points. \n";    
    // Entry control points
    // p0 = r0 + l0*getUnitVector(r0,r1)
    double l0 = getDistance(r0, r1);
    p0 = position_t(r0.x + l0*getUnitVector(r0,r1).x,r0.y + l0*getUnitVector(r0,r1).y);
    
    // p1 = r0 + l1*getUnitVector(r0,r1)
    double l1 = 0.85*l0;
    p1 = position_t(r0.x + l1*getUnitVector(r0,r1).x,r0.y + l1*getUnitVector(r0,r1).y);
    // p2 = i1 + l2*getUnitVector(i2,i1)
    double l2 =0.2;
    p2 = position_t(i1.x + l2*getUnitVector(i2,i1).x,i1.y + l2*getUnitVector(i2,i1).y);
    // p4 is calculated from the arc of the circunference l4 that separates vectors r0r1 and r0p4
    double l4 = 0.5;
    double phi_entry = l4/TRANSIT_RADIUS;
    p4 = position_t(TRANSIT_RADIUS*cos(atan2(r1.y-r0.y,r1.x-r0.x)+phi_entry)+r0.x, TRANSIT_RADIUS*sin(atan2(r1.y-r0.y,r1.x-r0.x)+phi_entry)+r0.y);
    // p3 = p4+l3*T. T is the tangent vector to the transit circunference p4
    position_t normal_entry = getUnitVector(p4,r0);
    position_t tangent_entry(normal_entry.y, -1*normal_entry.x); 
    double l3 = 0.2;
    p3 = position_t(p4.x + l3*tangent_entry.x, p4.y + l3*tangent_entry.y);
    // Exit control points (simetric to the entry calculation)
    // p4 is calculated from the arc of the circunference l5 that separates vectors r0r2 and r0p5
    double l5 = l4;
    double phi_exit = l5/TRANSIT_RADIUS;
    p5 = position_t(TRANSIT_RADIUS*cos(atan2(r2.y-r0.y,r2.x-r0.x)-phi_exit)+r0.x,TRANSIT_RADIUS*sin(atan2(r2.y-r0.y,r2.x-r0.x)-phi_exit)+r0.y);
    // p6 = p5+l6*T. T is the tangent vector to the transit circunference p5
    position_t normal_exit = getUnitVector(p5,r0);
    position_t tangent_exit(normal_exit.y, -1*normal_exit.x); 
    double l6 = l3;
    p6 = position_t(p5.x + l6*tangent_exit.x, p5.y + l6*tangent_exit.y);
    // p7 = i3 + l7*getUnitVector(i4,i3)
    double l7 = 0.10;
    p7 = position_t(i3.x + l7*getUnitVector(i4,i3).x, i3.y + l7*getUnitVector(i4,i3).y);
    // p8 = r2 + l8*getUnitVector(r0,r2)
    double l9 = getDistance(r0,r2);
    p9 = position_t(r0.x + l9*getUnitVector(r0,r2).x, r0.y + l9*getUnitVector(r0,r2).y);
    // p9 = r0 + l9*getUnitVector(r0,r2)
    double l8 = 0.85*l9;
    p8 = position_t(r0.x + l8*getUnitVector(r0,r2).x, r0.y + l8*getUnitVector(r0,r2).y);    
 }

#ifdef PRINT_MARKERS
void print_markers() {
    // Restriction points configuration
    visualization_msgs::Marker restriction_points_markers;
    visualization_msgs::Marker control_points_markers;
    restriction_points_markers.header.frame_id = control_points_markers.header.frame_id = "world";
    restriction_points_markers.header.stamp = control_points_markers.header.stamp = ros::Time::now();
    restriction_points_markers.ns = control_points_markers.ns = "points_markers";
    restriction_points_markers.action = control_points_markers.action = visualization_msgs::Marker::ADD;
    restriction_points_markers.pose.orientation.w = control_points_markers.pose.orientation.w = 1.0;
    
    restriction_points_markers.id = 0;
    control_points_markers.id = 1;

    restriction_points_markers.type = control_points_markers.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    restriction_points_markers.scale.x = control_points_markers.scale.x = 0.05;
    restriction_points_markers.scale.y = control_points_markers.scale.y = 0.05;

    geometry_msgs::Point p;
    position_t restriction_points_vector[] = {r0, r1, r2, i1, i2, i3, i4};
    for(int k = 0; k <= 6; k++) {
        std_msgs::ColorRGBA c;
        if (k < 3) {
            c.r = 1.0;
        }
        else if (k >=3 && k<5) {
            c.r = 1; c.g = 1;
        }
        else {
            c.r = 1.0; c.g = 0.5;
        }
        c.a = 1.0;
        p.x = restriction_points_vector[k].x;
        p.y = restriction_points_vector[k].y;
        p.z = 0.25;
        restriction_points_markers.points.push_back(p);
        restriction_points_markers.colors.push_back(c); 
    }

    geometry_msgs::Point q;
    position_t control_points_vector[] = {p0, p1, p2, p3, p4, p5, p6, p7, p8, p9};
    for(int kk = 0; kk <= 9; kk++) {
        std_msgs::ColorRGBA c;
        switch(kk) {
            case 0: c.g = 0.2; break;
            case 1: c.g = 0.4; break;
            case 2: c.g = 0.6; break;
            case 3: c.g = 0.8; break;
            case 4: c.g = 1.0; break;

            case 5: c.b = 0.2; break;
            case 6: c.b = 0.4; break;
            case 7: c.b = 0.6; break;
            case 8: c.b = 0.8; break;
            case 9: c.b = 1.0; break;
        }
        c.a = 1.0;
        q.x = control_points_vector[kk].x; q.y = control_points_vector[kk].y; q.z = 0.25;
        control_points_markers.points.push_back(q);
        control_points_markers.colors.push_back(c);
    }
    marker_pub.publish(restriction_points_markers);
    marker_pub.publish(control_points_markers);
}
void print_reference(position_t point) {
    // Restriction points configuration
    visualization_msgs::Marker reference_points;
    reference_points.header.frame_id = "world";
    reference_points.header.stamp = ros::Time::now();
    reference_points.ns = "reference_points";
    reference_points.action = visualization_msgs::Marker::ADD;
    reference_points.pose.orientation.w  = 1.0;
    
    reference_points.id = 2;
    
    reference_points.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    reference_points.scale.x = 0.05; reference_points.scale.y = 0.05;

    // Points are white
    reference_points.color.r = 1.0; reference_points.color.g = 1.0; reference_points.color.b = 1.0f;
    reference_points.color.a = 1.0;
    
    geometry_msgs::Point p;
    p.x = point.x; p.y = point.y; p.z = 0.25;
    reference_points.points.push_back(p);
    
    marker_pub.publish(reference_points);
}
#endif

void publishReference(position_t r) {
    #ifdef PRINT_MARKERS 
    print_reference(r);
    #endif
    reference_msg.x = r.x;
    reference_msg.y = r.y;
    reference_pub.publish(reference_msg);
}

void roundaboutReferenceGenerator() {
    float dist_ref = 0;
    float dist_control = 0;
    switch (maneuver_state) {
        case DEFINITION_STATE:
            cout << "Roundabout planner initialized. \n";
            selectRestrictionPoints();
            defineControlPoints();
            #ifdef PRINT_MARKERS
            print_markers();
            #endif
            maneuver_state = ENTRY_STATE;
            cout << "Starting entrance... \n";
            enable_control_msg.data = true;
            enable_control_pub.publish(enable_control_msg);
            speed_msg.data = 50;
            speed_pub.publish(speed_msg);
        break;
        case ENTRY_STATE:
            reference = bezierQuartic(p0,p1,p2,p3,p4,t);
            publishReference(reference);
            if(getDistance(vehicle_pose,p4) < lookahead || t > 1) {
                t = 0;
                maneuver_state = CIRCULATION_STATE;
                cout << "Circulating inside the roundabout...\n";
            }
            else {
                if (getDistance(vehicle_pose, reference) < lookahead) {
                t+= 0.025;
                // cout << "New reference generated. t = " << t << "\n";
                }
            }
            break;
        case CIRCULATION_STATE:
            reference = circunference(p4,r0,TRANSIT_RADIUS,t);
            publishReference(reference);
            if (getDistance(vehicle_pose, reference) < lookahead) {
                t+= 0.02;
                // cout << "New reference generated. \n" << "t = " << t << "\n";
            }
            if(getDistance(vehicle_pose,p5) < lookahead) {
                t = 0;
                maneuver_state = EXIT_STATE;
                cout << "Exiting roundabout...\n";
            }
            break;
        case EXIT_STATE:
            reference = bezierQuartic(p5,p6,p7,p8,p9,t);
            publishReference(reference);
            if (getDistance(vehicle_pose, reference) < lookahead) {
                t+= 0.025;
                // cout << "New reference generated. \n"<< "t = " << t << "\n";
            }
            if(getDistance(vehicle_pose,p9) < lookahead || t > 1) {
                t = 0.1;
                maneuver_state = IDLE_STATE;
            }
            break;
        case IDLE_STATE:
                cout << "Maneuver finished.\n";
                maneuver_state = DEFINITION_STATE;
                enable_roundabout_planner = false;
                enable_control_msg.data = false;
                enable_control_pub.publish(enable_control_msg);
                cout << "Roundabout planner disabled.\n";
            break;
    }
}
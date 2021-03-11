#include "crossing_planner.h"

// Global variables
maneuver_state_t maneuver_state = DEFINITION_STATE; // In which state is the car? Defining, entrying, circulating inside or exiting?

position_t r0(0,0), r1(0,0), r2(0,0), i1(0,0), i2(0,0), i3(0,0), i4(0,0);
position_t p0(0,0), p1(0,0), p2(0,0), p3(0,0), p4(0,0);
double t = 0.1;
position_t reference(0,0);

int main(int argc, char **argv) {
    // Node info
    ros::init(argc, argv, "crossing_planner");
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
    odom_sub = nh.subscribe(odometry_topic, ODOM_QUEUE_SIZE, callbackOdomData);
    adir_sub = nh.subscribe(planning_topic, PLANNING_QUEUE_SIZE, callbackADIRData);
    
    // Publications
    reference_pub = nh.advertise<adir::point2D>(reference_topic,1000);
    enable_control_pub = nh.advertise<std_msgs::Bool>(control_topic, 1000);
    speed_pub = nh.advertise<std_msgs::Int16>(speed_topic, 1000);
    
    #ifdef PRINT_MARKERS
        marker_pub = nh.advertise<visualization_msgs::Marker>(planning_markers_topic, MARKERS_QUEUE_SIZE);
    #endif

    // Frequency rate
    ros::Rate node_loop_rate(loop_rate);

    // Main loop
    while (ros::ok()) {
        if (enable_crossing_planner) { // If environment_class enables this node, let's start the state machine
            crossingReferenceGenerator();
        }
        ros::spinOnce();
        node_loop_rate.sleep();
    }
    return 0;
}

// Callback listening to the position of the car
 void callbackOdomData(const nav_msgs::Odometry::ConstPtr& msg) {
     vehicle_pose = position_t(msg -> pose.pose.position.x, msg -> pose.pose.position.y);
 }

// Callback listening to activation message from the environment classifier
void callbackADIRData(const adir::enable_t::ConstPtr& msg) {
    enable_crossing_planner = msg -> crossing;
    node_entry = msg -> node_entry;
    node_exit = msg -> node_exit;
}

// Linear aproximation between two scalars using Bezier equation
double bezierLinearScalar(double a, double b, double t) {
    return a + ((b-a)*t);
}

// Linear aproximation between two points using Bezier equation between scalars
position_t bezierLinear(position_t P, position_t Q, double t) {
    position_t R(0,0);
    R.x = bezierLinearScalar(P.x,Q.x,t);
    R.y = bezierLinearScalar(P.y,Q.y,t);
    return R;
}

// Fourth grade aproximation between five points using De Casteljau’s algorithm. Easier to see graphically.
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

// Euclidean distance between two points
double getDistance(position_t p, position_t q) {
     return sqrt((q.x - p.x)*(q.x - p.x)+(q.y - p.y)*(q.y - p.y));
 }

// Vector between two points
 position_t getVector(position_t p, position_t q) {
     position_t pq(q.x - p.x, q.y - p.y);
    return pq;
 }

// Unit vector between two points
position_t getUnitVector(position_t p, position_t q) {
     position_t pq = getVector(p,q);
     double dist = getDistance(p,q);
     position_t pq_unit(0,0);
     if(dist > 0.001) {
        pq_unit = position_t(pq.x/dist, pq.y/dist);
     }
     return pq_unit;
 }

// Choose restriction points of the roundabout depending on the entry/exit
 void selectRestrictionPoints() {
    r1 = vehicle_pose;
    switch(node_entry){
        // Curved crossing
        case 9: 
            i1 = position_t(-4.3139,-0.1492);
            i2 = position_t(-4.6042,0.1722);
            break;
        case 10:
            i1 = position_t(-4.2772,-0.7886);
            i2 = position_t(-3.9648,-0.5100);
            break;
        case 11: 
            i1 = position_t(-4.9182,-0.7556);
            i2 = position_t(-4.6046,-1.0749);
            break;
        // Regular crossing
        case 15: 
            i1 = position_t(-5.0695,-4.6454);
            i2 = position_t(-5.5000,-4.5650);
            break;
        case 16: 
            i1 = position_t(-4.6651,-5.0687);
            i2 = position_t(-4.6506,-4.6488);
            break;
        case 17: 
            i1 = position_t(-5.0717,-5.5027);
            i2 = position_t(-4.6447,-5.4963);
            break;
        default: 
            cout << "Exception on entry node. \n";
            break;
    }
    cout << "Restriction points of the entry created. \n";
    switch(node_exit){
        // Curved crossing
        case 12:
            i3 = position_t(-4.3139,-0.1492);
            i4 = position_t(-3.9933,-0.4689);
            r0 = position_t(-3.9648,-0.5100);
            r2 = position_t(-3.0611,-0.20);
            break;
        case 13: 
            i3 = position_t(-4.2772,-0.7886);
            i4 = position_t(-4.6049,-1.0619);
            if (node_entry == 9) {
                r0 = position_t(-3.9648,-0.5100);
            }
            else {
                r0 = position_t(-4.6046,-1.0749);
            }
            r2 = position_t(-3.6448,-1.7255);
            break;
        case 14: 
            i3 = position_t(-4.9182,-0.7556);
            i4 = position_t(-5.4006,-0.8654);
            r0 = position_t(-4.6046,-1.0749);
            r2 = position_t(-5.2838,-2.0571);
            break;
        // Regular crossing
        case 18: // right turn 
            i3 = position_t(-5.0695,-4.6454);
            i4 = position_t(-4.6459,-4.6512);
            r0 = position_t(-4.6506,-4.6488);
            r2 = position_t(-4.8383,-3.5583);
            
            break;
        case 19: 
            i3 = position_t(-4.6651,-5.0687);
            i4 = position_t(-4.6496,-5.5033);
            if (node_entry == 15) {
                r0 = position_t(-4.6506,-4.6488);
            }
            else {
                r0 = position_t(-4.6447,-5.4963);
            }
            r2 = position_t(-3.4673,-5.2620);
            break;
        case 20: 
            i3 = position_t(-5.0717,-5.5027);
            i4 = position_t(-5.4883,-5.4876);
            r0 = position_t(-4.6447,-5.4963);
            r2 = position_t(-5.2865,-6.5681);
            break;
        default: 
            cout << "Exception on exit node. \n";
            break;
    }
    cout << "Restriction points of the exit created. \n";
 }

// Calculate control points that will define the trajectory
 void defineControlPoints() {
    cout << "Defining control points. \n";    
    double radius, theta, l1;
    position_t iturn(0,0);
    // Left turns
    if( (node_entry == 17 && node_exit == 19) || 
        (node_entry == 11 && node_exit == 13) || 
        node_exit == 18 || node_exit == 12) {
    
        l1 = -1*getDistance(i1,i2)/3;
        radius = getDistance(i1,r0);
        theta = -M_PI_4;
        iturn = i1;
    }
    // Right turns
    else {
        l1 = getDistance(i1,i2);
        radius = getDistance(i2,r0);
        theta = M_PI_4;
        iturn = i2;
    }
    
    // Start point
    p0 = r1;
    // Thanks to this point the car doesnt walk out of the lane when turning, p1 = i1 + l1*getUnitVector(i2,i1)
    p1 = position_t(i1.x + l1*getUnitVector(i1,i2).x, i1.y + l1*getUnitVector(i1,i2).y);
    // Point inside the crossing, p2 is calculated from the arc of the circunference with center in r0 and starts in iturn with radius "radius"
    p2 = position_t(radius*cos(atan2(iturn.y-r0.y,iturn.x-r0.x)+theta)+r0.x, 
                    radius*sin(atan2(iturn.y-r0.y,iturn.x-r0.x)+theta)+r0.y);
    // Point in the center of the exit lane, p3 = i3 + l3*getUnitVector(i3,i4)
    double l3 = getDistance(i4,i3)/2;
    p3 = position_t(i3.x + l3*getUnitVector(i3,i4).x, i3.y + l3*getUnitVector(i3,i4).y);
    // End point
    p4 = r2;
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
        position_t control_points_vector[] = {p0, p1, p2, p3, p4};
        for(int kk = 0; kk <= 4; kk++) {
            std_msgs::ColorRGBA c;
            switch(kk) {
                case 0: c.g = 0.2; break;
                case 1: c.g = 0.4; break;
                case 2: c.g = 0.6; break;
                case 3: c.g = 0.8; break;
                case 4: c.g = 1.0; break;
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

void crossingReferenceGenerator() {
    switch (maneuver_state) {
        case DEFINITION_STATE:
            cout << "Crossing planner initialized. \n";
            selectRestrictionPoints();
            defineControlPoints();
            #ifdef PRINT_MARKERS
                print_markers(); // Print markers in the simulator
            #endif
            maneuver_state = CIRCULATION_STATE; // Go to next state
            enable_control_msg.data = true;
            enable_control_pub.publish(enable_control_msg); // Enable control node
            speed_msg.data = 50;
            speed_pub.publish(speed_msg); // Start moving
            cout << "Circulation inside the crossing... \n";
        break;
        case CIRCULATION_STATE:
            reference = bezierQuartic(p0,p1,p2,p3,p4,t); // Calculate reference using control points and t parameter
            publishReference(reference);
            if (getDistance(vehicle_pose, reference) < lookahead) { // If reference is reached, increment t to generate another one.
                t+= 0.025;
                // cout << "New reference generated. t = " << t << "\n";
            }
            if(getDistance(vehicle_pose,p4) < lookahead || t > 1) { // If entry is done
                t = 0;
                maneuver_state = IDLE_STATE; // Go to next state
            }
            break;
        case IDLE_STATE:
                cout << "Maneuver finished... \n";
                maneuver_state = DEFINITION_STATE; // Reset state machine
                enable_crossing_planner = false; // Auto-disable planner
                enable_control_msg.data = false; // Disable control
                enable_control_pub.publish(enable_control_msg);
                cout << "Crossing planner disabled. \n";
            break;
    }
}
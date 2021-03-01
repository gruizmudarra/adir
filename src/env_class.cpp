#include "env_class.h"

// Global variables
bool inside_roundabout = false; // Is the car already in a roundabout?
bool inside_crossing = false;   // Is the car already in a crossing?
bool ld_activated = false;      // Is the lane tracking node activated?
string prev_environment ="";    // For debugging environment classification
// int n = 0;

 int main (int argc, char **argv) {
    // Node info
    ros::init(argc, argv, "env_class");
	ros::NodeHandle nh;
    // Parameters
    nh.param<int>("/loop_rate", loop_rate, 50);
    nh.param<double>("/lookahead", lookahead, 0.01);
    nh.param<int>("/stop_speed", stop_speed, 0);
    nh.param<int>("/move_speed", move_speed, 70);
    nh.param<string>("/curvature_topic", curvature_topic, "/adir/curvature_calc");
    nh.param<string>("/odom_topic", odometry_topic, "/odom");
    nh.param<string>("/line_detection_topic", line_detection_topic, "/tracking_enable");
    nh.param<string>("/planning_topic", planning_topic, "/adir/enable_planning");
    nh.param<string>("/environment_topic", environment_topic, "/adir/environment");
    nh.param<string>("/speed_topic", speed_topic, "/manual_control/speed");
    
    // Subcriptions
    curv_sub = nh.subscribe(curvature_topic, CURV_QUEUE_SIZE, callbackCurvData);
    odom_sub = nh.subscribe(odometry_topic, ODOM_QUEUE_SIZE, callbackOdomData);
    
    // Publications
    tracking_pub = nh.advertise<std_msgs::Bool>(line_detection_topic, LD_QUEUE_SIZE);
    adir_pub = nh.advertise<adir::enable_t>(planning_topic, PLANNING_QUEUE_SIZE);
    env_pub = nh.advertise<std_msgs::String>(environment_topic, ENV_QUEUE_SIZE);
    speed_pub = nh.advertise<std_msgs::Int16>(speed_topic, SPEED_QUEUE_SIZE);

    // Define topologic map of the circuit
    std::vector<position_t> topologic_map = defineTopologicMap();

    // Frequency rate
    ros::Rate node_loop_rate(loop_rate);

    // Main loop
    while (ros::ok()) {

        environmentClassifier(topologic_map);
        
        ros::spinOnce();
        node_loop_rate.sleep(); // sleep the rest of the period
    }
    return 0;
 }

// Callback listening to curvature data of all the lanes
 void callbackCurvData(const adir::curvature_t::ConstPtr& msg) {
    curvLane.left = msg -> left;
    curvLane.center = msg -> center;
    curvLane.right = msg -> right;
 }

// Callback listening to the position of the car
 void callbackOdomData(const nav_msgs::Odometry::ConstPtr& msg) {
     vehicle_pose.x = msg -> pose.pose.position.x;
     vehicle_pose.y = msg -> pose.pose.position.y; 
     //cout << "Vehicle pose -> x: " << vehicle_pose.x << " y: " << vehicle_pose.y << endl;
 }

// Calculates the euclidean distance from p to q
 double getDistance(position_t p, position_t q) {
     return sqrt((q.x - p.x)*(q.x - p.x) + (q.y - p.y)*(q.y - p.y));
 }

// Define cartesian coordinates of every intersection, each
std::vector<position_t> defineTopologicMap(){
    // Cartesian coordinates matrix of each node (dimension: 2xn_nodes) 
    std::vector<position_t> nodes_matrix;
    //Roundabout 
    //Entries:
    nodes_matrix.push_back(position_t( 2.0095,-3.72014));  //1
    nodes_matrix.push_back(position_t(-1.84528,-3.550749)); //2
    nodes_matrix.push_back(position_t(-2.37184,-5.291795)); //3
    nodes_matrix.push_back(position_t(2.0339,-5.9971));  //4
    //Exits
    nodes_matrix.push_back(position_t(-1.9152,-2.8758)); //5
    nodes_matrix.push_back(position_t(-2.9942,-4.8727)); //6
    nodes_matrix.push_back(position_t(2.42829,-6.6881));  //7
    nodes_matrix.push_back(position_t(2.6465,-3.8140));  //8
    //Curved Crossing 
    //Entries:
    nodes_matrix.push_back(position_t(-3.5204603672, 0.2)); //9
    nodes_matrix.push_back(position_t(-3.690855,-1.07504)); //10
    nodes_matrix.push_back(position_t(-4.87123,-1.58464717865)); //11
    //Exits:
    nodes_matrix.push_back(position_t(-3.5204603672, -0.2)); // 12
    nodes_matrix.push_back(position_t(-3.99613, -1.377046));    //13
    nodes_matrix.push_back(position_t(-5.28350496292, -1.58464717865)); // 14
    //Regular Crossing:
    //Entries:
    nodes_matrix.push_back(position_t(-5.2882,-4.33024597168));   //15
    nodes_matrix.push_back(position_t(-4.1720,-4.86417)); //16
    nodes_matrix.push_back(position_t(-4.9159,-5.7948));   //17
    //Exits:
    nodes_matrix.push_back(position_t(-4.81341934204,-4.33024597168)); // 18
    nodes_matrix.push_back(position_t(-4.1720,-5.30747));  //19
    nodes_matrix.push_back(position_t(-5.2837767601,-5.79377508163)); // 20

    return nodes_matrix;
}

// Gives each node of the topologic map matrix a high level definition
string intersectionClassifier(int node_id) {
    string s = "";
    if (node_id >= 1 && node_id <= 20) {
        if (node_id >= 1 && node_id <=4) {
            s = "ROUNDABOUT ENTRY"; 
        }
        else if (node_id >= 5 && node_id <=8) {
            s = "ROUNDABOUT EXIT";
        }
        else if ((node_id >= 9 && node_id <=11) || (node_id >= 15 && node_id <=17)) {
            s = "CROSSING ENTRY";
        }
        else {
            s = "CROSSING EXIT";
        }
    }
    else {
       s = "UNKNOWN INTERSECTION";
    }
    return s;
}

// Checks if current position is near any node of the topologic map
// If it is, it returns a positive boolean value and store the node that
// is nearby. Else, it retuns a negative boolean. 
bool checkPosition(std::vector<position_t> map, int& node_id) {
    position_t node(0,0);
    double dist;
    for(int i = 0; i < map.size(); i++) {
        node = map[i];
        dist = getDistance(vehicle_pose,node);
        if (dist <= lookahead) {
            node_id = i+1;
            return true;
        }
    }
    return false;
}

// Main algorithm
void environmentClassifier(std::vector<position_t> map) {
    string environment = "";
    int node_id;
    // Checking if the car is near a intersection   
    bool intersection = checkPosition(map,node_id); 
    // If it is near an intersection, get what kind of intersection is.
    if (intersection) {
        environment = intersectionClassifier(node_id);
        // Given an intersection type, publish actuation
        if(environment == "ROUNDABOUT ENTRY" && !inside_roundabout) {
            // The car stops
            speed_msg.data = stop_speed;
            speed_pub.publish(speed_msg);
            // line_detection_fu deactivated
            tracking_msg.data = false;
            tracking_pub.publish(tracking_msg);
            ld_activated = false;
            // Ask exit node
            cout << "You are right now in the entry node " << node_id << " of the roundabout.\n";
            int node_exit = 0;
            while (node_exit < 5 || node_exit > 8) {
                cout << "Insert exit node (5-8): \n" ;
                cin >> node_exit;
                if (node_exit < 5 || node_exit > 8) {
                    cout << "Invalid node \n";
                }
            }
            // Publish info detected
            prev_environment = environment;
            env_msg.data = environment;
            env_pub.publish(env_msg);
            
            // Package info and activate the roundabout_planner node
            adir_msg.roundabout = true; // The car is in a roundabout
            adir_msg.crossing = false; 
            adir_msg.node_entry = node_id; // Entry node
            adir_msg.node_exit = node_exit; // Exit node
            adir_pub.publish(adir_msg);
            inside_roundabout = true;
        }
        else if (environment == "CROSSING ENTRY" && !inside_crossing) {
            // The car stops
            speed_msg.data = stop_speed;
            speed_pub.publish(speed_msg);
            // line_detection_fu deactivated
            tracking_msg.data = false;
            tracking_pub.publish(tracking_msg);
            ld_activated = false;
            // Ask exit node
            cout << "You are right now in the entry node " << node_id << " of a crossing.\n";
            int node_exit = 0;
            while (node_exit < 9 || node_exit > 20) {
                cout << "Insert exit node (12-13-14) or (18-19-20): \n" ;
                cin >> node_exit;
                if (node_exit <9 || node_exit > 20) {
                    cout << "Invalid node \n";
                }
            }
            // Don't want to turn, then no need to plan movement
            if ((node_id == 9 && node_exit == 14)   || // curved crossing
                (node_id == 11 && node_exit == 12)  ||
                (node_id == 15 && node_exit == 20)  || // regular crossing
                (node_id == 17 && node_exit == 18)) {
                // line_detection_fu activate
                tracking_msg.data = true;
                tracking_pub.publish(tracking_msg);
                ld_activated = true;
                // Keep going straight
                speed_msg.data = move_speed;
                speed_pub.publish(speed_msg);
            }
            // You want to turn, planning needed
            else {
                // Crossing_planner activated
                adir_msg.roundabout = false;
                adir_msg.crossing = true; // The car is in a crossing
                adir_msg.node_entry = node_id; // Entry node 
                adir_msg.node_exit = node_exit; // Exit node
                adir_pub.publish(adir_msg);
            }
            // Publish environment for debug
            prev_environment = environment;
            env_msg.data = environment;
            env_pub.publish(env_msg);
            inside_crossing = true;
        }
        // If the car has ended the movement inside a roundabout or crossing
        else if (((environment == "ROUNDABOUT EXIT") || 
                 (environment == "CROSSING EXIT")) && 
                 (inside_roundabout || inside_crossing)) {  
            // Environment debug
            prev_environment = environment;
            env_msg.data = environment;
            env_pub.publish(env_msg);
            // line_detection_fu reactivated
            tracking_msg.data = true;
            tracking_pub.publish(tracking_msg);
            ld_activated = true;
            // recover normal speed
            speed_msg.data = move_speed;
            speed_pub.publish(speed_msg);
            inside_roundabout = false;
            inside_crossing = false;
        }
    }
    // If it is not near a intersection, check curvature to check if the car is in a straight or curved road
    else {
        double curv_values[2] = {abs(curvLane.center), abs(curvLane.right)};
        // If curvature in both lane is below threshold, then it's a straight road
        if (curv_values[0] <= CURV_THRESHOLD && curv_values[1] <= CURV_THRESHOLD) { 
            environment = "STRAIGHT";
        }
        // If is lesser than a big number (inf)
        else {
            // If it's not inf, it's a curved road
            if (curv_values[0] < 100*CURV_THRESHOLD || curv_values[1] < 100*CURV_THRESHOLD) {
                // Negative -> Left
                if(curvLane.center < 0 || curvLane.right < 0) {
                    environment = "LEFT CURVE";
                }
                // Positive -> Right
                else {
                    environment = "RIGHT CURVE";
                }
            }
            // If it's inf, lane detection failed
            else {
                environment = "EXCEPTION";
            }
        }

        // Activate line detection if the car is not in a intersection
        if (!ld_activated && !inside_roundabout && !inside_crossing) {
            // line_detection_fu activated
            tracking_msg.data = true;
            tracking_pub.publish(tracking_msg);
            ld_activated = true;
        }
        else {
            // If there's a change, publish new environment detected
            if (environment != prev_environment && !inside_roundabout && !inside_crossing) {
                prev_environment = environment;
                env_msg.data = environment;
                env_pub.publish(env_msg);
            }
        }
    }
}
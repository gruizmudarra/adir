#include "env_class.h"

static const uint32_t LOOP_RATE = 10; // Hz
static const uint32_t ODOM_QUEUE_SIZE = 1;
static const uint32_t ENV_QUEUE_SIZE = 1000;
static const uint32_t CURV_QUEUE_SIZE = 1000;

static const uint32_t CURV_LIMIT = 300;

curvature_t curvLane;
position_t vehicle_pose(0,0); 

ros::Publisher env_pub;
std_msgs::String env_msg;

 int main (int argc, char **argv) {
    // Node info
    ros::init(argc, argv, "env_class");
	ros::NodeHandle nh;
    
    // Subcriptions
    ros::Subscriber curv_sub = nh.subscribe("/curvature_calc/array", CURV_QUEUE_SIZE, cb_curvData);
    ros::Subscriber odom_sub = nh.subscribe("/odom_ground_truth", ODOM_QUEUE_SIZE, cb_odomData);
    
    // Publications
    env_pub = nh.advertise<std_msgs::String>("/env_class", ENV_QUEUE_SIZE);
    
    // Define topologic map
    std::vector<position_t> topologic_map = define_intersection_nodes();

    ros::Rate node_loop_rate(LOOP_RATE);

    // Main loop
    while (ros::ok()) {
        environment_classifier(topologic_map);
        env_pub.publish(env_msg);
        ros::spinOnce();
        node_loop_rate.sleep();
    }
    return 0;
 }

 void cb_curvData(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    std::vector<float> temp_data;
    for (int i=0; i < msg-> data.size(); i++) {
        temp_data.push_back(msg -> data[i]);
    }
    curvLane.left = temp_data[0];
    curvLane.center = temp_data[1];
    curvLane.right = temp_data[2];
 }

 void cb_odomData(const nav_msgs::Odometry::ConstPtr& msg) {
     vehicle_pose.x = msg -> pose.pose.position.x;
     vehicle_pose.y = msg -> pose.pose.position.y; 
     cout << "Vehicle pose -> x: " << vehicle_pose.x << " y: " << vehicle_pose.y << endl;
 }

 double get_distance(position_t p, position_t q) {
     double dif_x = q.x - p.x;
     double dif_y = q.y - p.y;
     return sqrt(pow(dif_x, 2.0)+pow(dif_y,2.0));
 }

// Define cartesian coordinates of the point to turn
std::vector<position_t> define_intersection_nodes(){
    // Cartesian coordinates matrix of each node
    std::vector<position_t> nodes_matrix;
    //Roundabout 
    //Entries:
    nodes_matrix.push_back(position_t( 2.0095,-3.72014));  //1
    nodes_matrix.push_back(position_t(-1.84528,-3.550749)); //2
    nodes_matrix.push_back(position_t(-2.37184,-5.291795)); //3
    nodes_matrix.push_back(position_t(2.0339,-5.9971));  //4
    //Exits
    nodes_matrix.push_back(position_t(1.5667, -5.1082));  //5
    nodes_matrix.push_back(position_t(-0.42097, -3.55602)); //6
    nodes_matrix.push_back(position_t(-1.404672, -4.3537)); //7
    nodes_matrix.push_back(position_t(0.99706, -6.35349));  //8
    //Curved Crossing 
    //Entries:
    nodes_matrix.push_back(position_t(-3.5204603672, 0.2)); //9
    nodes_matrix.push_back(position_t(-3.690855,-1.07504)); //10
    nodes_matrix.push_back(position_t(-4.87123,-1.58464717865)); //11
    //Exits:
    nodes_matrix.push_back(position_t(-3.5204603672, -0.2)); // 15
    nodes_matrix.push_back(position_t(-3.99613, -1.377046));    //16
    nodes_matrix.push_back(position_t(-5.28350496292, -1.58464717865)); // 17
    //Regular Crossing:
    //Entries:
    nodes_matrix.push_back(position_t(-5.2882,-4.33024597168));   //12
    nodes_matrix.push_back(position_t(-4.1720,-4.86417)); //13
    nodes_matrix.push_back(position_t(-4.9159,-5.7948));   //14
    //Exits:
    nodes_matrix.push_back(position_t(-4.81341934204,-4.33024597168)); // 18
    nodes_matrix.push_back(position_t(-4.1720,-5.30747));  //19
    nodes_matrix.push_back(position_t(-5.2837767601,-5.79377508163)); // 20

    return nodes_matrix;
}

string intersection_class(int node_id) {
    string s = "";
    switch(node_id) {
        case 1: s = "ROUNDENTRY1"; 
                break;
        case 2: s = "ROUNDENTRY2"; 
                break;
        case 3: s = "ROUNDENTRY3"; 
                break;
        case 4: s = "ROUNDENTRY4"; 
                break;

        case 5: s = "ROUNDEXIT5"; 
                    break;
        case 6: s = "ROUNDEXIT6"; 
                break;
        case 7: s = "ROUNDEXIT7"; 
                break;
        case 8: s = "ROUNDEXIT8"; 
                break;

        case 9: s = "CCROSSING ENTRY9"; 
                break;
        case 10: s = "CCROSSING ENTRY10"; 
                break;
        case 11: s = "CCROSSING ENTRY11"; 
                break;

        case 12: s = "CCROSSING EXIT15"; 
                break;
        case 13: s = "CCROSSING EXIT16"; 
                break;
        case 14: s = "CCROSSING EXIT17"; 
                break;

        case 15: s = "RCROSSING ENTRY12"; 
                break;
        case 16: s = "RCROSSING ENTRY13"; 
                break;
        case 17: s = "RCROSSING ENTRY14"; 
                break;

        case 18: s = "RCROSSING EXIT18"; 
                break;
        case 19: s = "RCROSSING EXIT19"; 
                break;
        case 20: s = "RCROSSING EXIT20"; 
                break;
        default: s = "UNKNOWN INTERSECTION"; 
                break;
    }
    return s;
}

bool check_position(std::vector<position_t> map, int& node_id) {
    position_t node(0,0);
    double dist;
    for(int i = 0; i < map.size(); i++) {
        node = map[i];
        dist = get_distance(vehicle_pose,node);
        if (dist <= 0.25) {
            node_id = i+1;
            return true;
        }
    }
    return false;
}

void environment_classifier(std::vector<position_t> map) {
    string environment = "";
    int node_id;
    // Checking if the car is near (0.25 m) of a intersection   
    bool intersection = check_position(map,node_id); 
    
    // If it is near an intersection, get what node is in the topologic map
    if (intersection) {
        environment = intersection_class(node_id);       
    }
    // If it is no near a intersection, check curvature to check if the car is in a straight or curved road
    else {
        double mod_curv[2] = {abs(curvLane.center), abs(curvLane.right)};
        // If curvature is greater than 300, then it's a straight road
        if (mod_curv[0] > CURV_LIMIT || mod_curv[1] > CURV_LIMIT) { 
            environment = "STRAIGHT";
        }
        // If is lesser or equal than +-300
        else {
            // If it's not zero, it's a curved road
            if (mod_curv[0] || mod_curv[1]) {
                // Negative -> Left
                if(curvLane.center < 0 || curvLane.right < 0) {
                    environment = "LEFT CURVE";
                }
                // Positive -> Right
                else {
                    environment = "RIGHT CURVE";
                }
            }
            // If it's zero, lane detection failed
            else {
                environment = "EXCEPTION";
            }
        }
    }
    env_msg.data = environment;
}
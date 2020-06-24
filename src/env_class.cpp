#include "env_class.h"

static const uint32_t MY_ROS_QUEUE_SIZE = 1000;

curvature_t curvLane;
position_t vehicle_pose(0,0); 

ros::Publisher env_pub;
std_msgs::String env_msg;

 int main (int argc, char **argv) {
    // Node info
    ros::init(argc, argv, "env_class");
	ros::NodeHandle nh;
    
    // Subcriptions
    ros::Subscriber curv_sub = nh.subscribe("/curvature_calc/array", MY_ROS_QUEUE_SIZE, cb_curvData);
    ros::Subscriber odom_sub = nh.subscribe("/odom", MY_ROS_QUEUE_SIZE, cb_odomData);
    
    // Publications
    env_pub = nh.advertise<std_msgs::String>("/env_class", MY_ROS_QUEUE_SIZE);
    
    // Define topologic map
    std::vector<position_t> topologic_map = define_intersection_nodes();

    // Main loop
    while (ros::ok()) {
        environment_classifier(topologic_map);
        env_pub.publish(env_msg);
        ros::spinOnce();
        sleep(1);
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
 }

 double get_distance(position_t p1, position_t p2) {
     double dif_x = p1.x - p2.x;
     double dif_y = p1.y - p2.y;
     return (dif_x*dif_x)+(dif_y*dif_y);
 }

// Define cartesian coordinates of the point to turn
std::vector<position_t> define_intersection_nodes(){
    // Cartesian coordinates matrix of each node
    std::vector<position_t> nodes_matrix;
    //Roundabout 
    //Entries:
    nodes_matrix.push_back(position_t( 1.72537,-3.9271));  //1
    nodes_matrix.push_back(position_t(-1.46146,-3.86171)); //2
    nodes_matrix.push_back(position_t(-1.84672,-5.23007)); //3
    nodes_matrix.push_back(position_t(1.75858,-5.81786));  //4
    //Exits
    nodes_matrix.push_back(position_t(-0.674353599548, -3.6364672184));  //5
    nodes_matrix.push_back(position_t(-1.48543524742, -4.52174901962)); //6
    nodes_matrix.push_back(position_t(1.07619309425, -6.2709069252)); //7
    nodes_matrix.push_back(position_t(1.59637641907, -4.8202419281));  //8
    //Curved Crossing 
    //Entries:
    nodes_matrix.push_back(position_t(-3.8876,0.1944)); //9
    nodes_matrix.push_back(position_t(-4.05578,-0.76218)); //10
    nodes_matrix.push_back(position_t(-4.87123,-1.24748)); //11
    //Exits:
    nodes_matrix.push_back(position_t(-3.5204603672, -0.246602073312)); // 15
    nodes_matrix.push_back(position_t(-3.705, -1.6246));    //16
    nodes_matrix.push_back(position_t(-5.28350496292, -1.58464717865)); // 17
    //Regular Crossing:
    //Entries:
    nodes_matrix.push_back(position_t(-5.2882,-4.6787));   //12
    nodes_matrix.push_back(position_t(-4.51369,-4.86417)); //13
    nodes_matrix.push_back(position_t(-4.9159,-5.7948));   //14
    //Exits:
    nodes_matrix.push_back(position_t(-4.81341934204,-4.33024597168)); // 18
    nodes_matrix.push_back(position_t(-3.7626,-5.30747));  //19
    nodes_matrix.push_back(position_t(-5.2837767601,-5.79377508163)); // 20

    return nodes_matrix;
}

string intersection_class(int node_id) {
    string s = "";
    switch(node_id) {
        case 1: s = "ROUNDABOUT ENTRY"; 
                break;
        case 2: s = "ROUNDABOUT ENTRY"; 
                break;
        case 3: s = "ROUNDABOUT ENTRY"; 
                break;
        case 4: s = "ROUNDABOUT ENTRY"; 
                break;

        case 5: s = "ROUNDABOUT EXIT"; 
                    break;
        case 6: s = "ROUNDABOUT EXIT"; 
                break;
        case 7: s = "ROUNDABOUT EXIT"; 
                break;
        case 8: s = "ROUNDABOUT EXIT"; 
                break;

        case 9: s = "CCROSSING ENTRY"; 
                break;
        case 10: s = "CCROSSING ENTRY"; 
                break;
        case 11: s = "CCROSSING ENTRY"; 
                break;

        case 12: s = "CCROSSING EXIT"; 
                break;
        case 13: s = "CCROSSING EXIT"; 
                break;
        case 14: s = "CCROSSING EXIT"; 
                break;

        case 15: s = "RCROSSING ENTRY"; 
                break;
        case 16: s = "RCROSSING ENTRY"; 
                break;
        case 17: s = "RCROSSING ENTRY"; 
                break;

        case 18: s = "RCROSSING EXIT"; 
                break;
        case 19: s = "RCROSSING EXIT"; 
                break;
        case 20: s = "RCROSSING EXIT"; 
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
        if (dist <= 0.5) {
            node_id = i+1;
            return true;
        }
    }
    return false;
}

void environment_classifier(std::vector<position_t> map) {
    string environment = "";
    int node_id;   
    bool intersection = check_position(map,node_id); 
    
    if (intersection) {
            environment = intersection_class(node_id);
    }
    else {
        if (abs(curvLane.center) < 300 || abs(curvLane.center) < 300) {
            environment = "CURVE";
            if(curvLane.center < 0 || curvLane.right < 0) {
                environment = "LEFT " + environment;
            }
            else if(curvLane.center > 0 || curvLane.right > 0) {
                environment = "RIGHT " + environment;
            }
        }
        else if (abs(curvLane.center) > 300 || abs(curvLane.right) > 300) {
            environment = "STRAIGHT";
        }
        else
        {
            environment = "UNCERTAIN";
        }
    }
    
    /*
    if (!curvLane.center && !curvLane.right) {
        intersection = check_position(map,node_id);
        if (intersection) {
            environment = intersection_class(node_id);
        }
        else {
            environment = "LINE DETECTION FAILED";
        }
    }
    else if (abs(curvLane.center) < 300 || abs(curvLane.center) < 300) {
        environment = "CURVE";
        if(curvLane.center < 0 || curvLane.right < 0) {
            environment = "LEFT " + environment;
        }
        else if(curvLane.center > 0 || curvLane.right > 0) {
            environment = "RIGHT " + environment;
        }
    }
    else if (abs(curvLane.center) > 300 || abs(curvLane.right) > 300) {
        environment = "STRAIGHT";
    }
    else
    {
        environment = "UNCERTAIN";
    }
    */
    env_msg.data = environment;
}

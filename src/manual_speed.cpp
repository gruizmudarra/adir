#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
#include "iostream"

using namespace std;

int16_t v = 0;
std_msgs::Int16 v_msg;
ros::Publisher v_pub;

uint8_t steer = 0;
std_msgs::UInt8 steer_msg;
ros::Publisher steer_pub;

bool enable = true;
std_msgs::Bool enable_msg;
ros::Publisher enable_pub;


// Variable para terminar la ejecución al pulsar x
bool salir=false;

void get_speed_from_key(char input);

int main(int argc, char **argv) {
    // Se inicia el nodo keyboard
    ros::init(argc, argv, "manual_speed");

    // Se inicia el manejador del nodo
    ros::NodeHandle nh;

    // Variable para publicar por /cmd_key un mensaje de longitud 1000
    v_pub = nh.advertise<std_msgs::Int16>("/manual_control/speed", 1000);
    steer_pub = nh.advertise<std_msgs::UInt8>("/steering", 1000);
    enable_pub = nh.advertise<std_msgs::Bool>("/tracking_enable", 1000);

    // Objeto de ROS con la frecuencia del bucle
    ros::Rate loop_rate(100);
    
    // El modo raw permite escribir por teclado sin presionar enter
    system("stty raw");
    
    // Mientras este lanzado ROS y no se pulse salir, se ejecuta el bucle
    while(ros::ok() && !salir)  {
        char input=getchar(); // Variable con la tecla pulsada
        
        // En función de la tecla puslada, se guarda un String con la orden a realizar
        get_speed_from_key(input);
        ros::spinOnce();
        // Frecuencia de ejecución del bucle (en este caso 1 H
        loop_rate.sleep();
    }
    // Se resetea el terminal y lo deja en modo inicial
    system("stty cooked");
    return 0;
}

void get_speed_from_key(char input) {
    // En función de la tecla puslada, se guarda un String con la orden a realizar
    switch(input) {
            case 'w': 
                // GoForward
                enable = true;
                v = 100;
                steer = 90;
                break;
            case 's': 
                // Stop
                enable = false;
                v = 0;
                steer = 90;
                break;
            case 'x': 
                // Backwards
                enable = false;
                v = -70;
                steer = 90;
                break;
            case 'd':
                // Right 
                enable = false;
                v = 50;
                steer = 220;
                break;
            case 'a': 
                // Left 
                enable = false;
                v = 50;
                steer = 0;
                break;

            case 'c':{salir=true; break;} // Reinicia la consola
        }
    v_msg.data = v;
    steer_msg.data = steer;
    enable_msg.data = enable;
    // cout << " Publishing: " << v_msg << " , " << steer_msg << " , " << enable_msg << "\n";
    
    // Publishing
    v_pub.publish(v_msg);
    steer_pub.publish(steer_msg);
    enable_pub.publish(enable_msg);

}
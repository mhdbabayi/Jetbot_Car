#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <termios.h>

#include <stdio.h>
#include <signal.h>

// for printing
#include <iostream>

static volatile sig_atomic_t keep_running = 1;
std::string radius_topic;

void sigHandler(int not_used) {
    keep_running = 0;
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "keyboard");
    // Initialize Node Handle
    ros::NodeHandle n = ros::NodeHandle("~");
    std_msgs::Float64 initialMessage;
    // Initialze publisher
    std::string keyboard_topic;
    n.getParam("keyboard_topic", keyboard_topic);
    n.getParam("radius_topic" , radius_topic);
    ros::Publisher key_pub = n.advertise<std_msgs::String>(keyboard_topic, 10);
    ros::Publisher radiusPub = n.advertise<std_msgs::Float64>(radius_topic , 10);
    initialMessage.data = 10.0;
    radiusPub.publish(initialMessage);
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON);
    tcsetattr( STDIN_FILENO, 0, &newt);

    struct sigaction act;
    act.sa_handler = sigHandler;
    sigaction(SIGINT, &act, NULL);
    

    std_msgs::String msg;
    int c;
    while ((ros::ok()) && (keep_running)) {
        // get the character pressed
        c = getchar();

        // Publish the character 
        msg.data = c;
        key_pub.publish(msg);
    }

    tcsetattr( STDIN_FILENO, 0, &oldt);
    
    return 0;
}
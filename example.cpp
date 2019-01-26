#include <ros/ros.h>
#include "joy.hpp"
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "joy_interface");
    ros::NodeHandle node;
    ros::Publisher PubCmdVel = node.advertise<geometry_msgs::Twist>("/cmd_vel", 2);
    bir::JoyController Controller(node);

    while(ros::ok()){
        geometry_msgs::Twist msg;
        float linear = 0.00, angular = 0.00;
        if(Controller.get().button.A) {
            linear = 0.5;
            if(Controller.get().axes.RT <= 0.00){
                linear += -(Controller.get().axes.RT);
            }
        }
        msg.linear.x = linear;
        msg.angular.z = Controller.get().axes.horizontal_L_stick;
        PubCmdVel.publish(msg);
    }
}

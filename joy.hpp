#ifndef JOY_XBOX_INTERFACE_HPP
#define JOY_XBOX_INTERFACE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <string>

namespace bir {
    class JoyController {
    
        public:
            explicit JoyController(const ros::NodeHandle&);
            explicit JoyController(const ros::NodeHandle&, const std::string&);

            virtual ~JoyController();
        private:
            ros::NodeHandle _Node;
            ros::Subscriber _SubJoy;
            std::string     _namespace;
            std::string     _joyTopicName;
            typedef struct {
                bool A;     bool B;     bool X;     bool Y;
                bool LB;    bool RB;    bool back;  bool start;
                bool power; bool L3;    bool R3;
            } Buttons;
            
            enum BUTTONS {
                A,
                B,
                X,
                Y,
                LB,
                RB,
                BACK,
                START,
                POWER,
                L3
            };

            typedef struct {
                double horizontal_L_stick;
                double vertical_L_stick;
                double LT;
                double horizontal_R_stick;
                double vertical_R_stick;
                double RT;
                double horizontal_crosskey;
                double vertical_crosskey;
            } Axes;

            enum AXES {
                HORIZONTAL_L_STICK,
                VERTICAL_L_STICK,
                LT,
                HORIZONTAL_R_STICK,
                VERTICAL_R_STICK,
                RT,
                HORIZONTAL_CROSSKEY,
                VERTICAL_CROSSKEY            
            };
        public:
            typedef struct {
                Buttons button;
                Axes axes;
            } Joy;
            
        private:
            Joy _Joy;
            /*  
                Function: subJoyCallback
                Type: void
                Params: const sensor_msgs::JoyConstPtr&
                Return: Nothing
                Duty: Recive data from Joy Topic and update _Joy variable
            */
            void subJoyCallback(const sensor_msgs::JoyConstPtr&);
            /*  
                Function: initJoyController
                Type: void
                Params: void
                Return: Nothing
                Duty: Initialize all Joy default configs, Subscribers
            */
            void initJoyController();
        public:
            /*  
                Function: setJoyTopicName
                Type: void
                Params: std::string
                Return: Nothing
                Duty: Set the variable _joyTopicName, Variable whose indicate the topic of JoyController.
            */
            void setJoyTopicName(const std::string&);
            /*  
                Function: get
                Type: bir::JoyController::Joy
                Params: Nothing
                Return: _Joy current status
                Duty: Return Joy Controller current status.
            */
            const Joy& get();

    };
}

#endif
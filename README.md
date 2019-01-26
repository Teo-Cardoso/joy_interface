# Joy Interface
Library to use Xbox Controller using ROS

# Getting Start
  Include joy interface header
  
  >#include <joy.hpp>

# How to use
  Instantiate `bir::JoyController` class
  
  Example:
  >bir::JoyController Controller(node, namespace);
  
  Use the method `bir::JoyController::get()` to acess internal Joy Status whose is automatic update on each `ros::spin()` or `ros::spinOnce()`
    
  Example:
  >bir::JoyController Controller(node, namespace);
  
  >...
  
  > bool button_A = Controller.get().button.A;

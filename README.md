# Friction_Finger_low_level_control
  This repository contains the low level controller code and high level actions to interface with the friction finger gripper.
  This is a ROS package written in c++ and python.
  
## Code Organization:
  * include 
      * finger.hpp - Finger class declaration
      * controller_client.hpp - Function declaration of High level finger actions
  * src
      * controller.cpp - Initializing the controller with 2 fingers
      * High level controller.cpp - Creating a service architecture for each high level actions
  * scripts
      * Finger planning.py - Control code for sliding and rotating objects
  * srv
      The rosservice files used for communicatng between the service and client.
      * Holdcommand.srv  
      * positioncommand.srv
      * SendFrictionSrface.srv
  ## Experiment Setup
  ![Experiment](https://github.com/gokul-gokz/Friction_Finger_low_level_control/blob/master/Images/Vf_system.jpg)
      
  ## Friction Finger Gripper
  
  ![Friction_Finger_Gripper](https://github.com/gokul-gokz/Friction_Finger_low_level_control/blob/master/Images/Frcition_finger%20gripper.png)
  
  ## For Sliding an object
  
  ![Sliding_object](https://github.com/gokul-gokz/Friction_Finger_low_level_control/blob/master/Images/Sliding.png)
  
  ## For Rotating an object
  
  ![Rotate object](https://github.com/gokul-gokz/Friction_Finger_low_level_control/blob/master/Images/Actuation.png)

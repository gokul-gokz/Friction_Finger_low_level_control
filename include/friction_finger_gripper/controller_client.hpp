#include <friction_finger_gripper/finger.hpp>
#include <array>
#include <std_msgs/Bool.h>
#include <common_msgs_gl/SendBool.h>
#include <common_msgs_gl/SendDoubleArray.h>


bool set_friction_l=false;
bool set_friction_r=false;
bool set_modes,send_pos,send_torque,read_pos;

float read_position(ros::NodeHandle n,int motor_num)
{
 //ros::NodeHandle n;
  ros::service::waitForService("read_pos");
  ros::ServiceClient client_read_pos=n.serviceClient<common_msgs_gl::GetDoubleArray>("read_pos");
  common_msgs_gl::GetDoubleArray srv;
  
   if(client_read_pos.call(srv))
  {
   
   ROS_INFO("Read_position_success");
   //ROS_INFO("%f",srv.response.data[0]);
   return srv.response.data[motor_num-1];
  }
  else
  {
  ROS_INFO("Failure");
  return -1;
 }
}


bool set_actuator_modes(ros::NodeHandle n,int size, int modes[])
{
 //ros::NodeHandle n;
 ros::service::waitForService("set_operating_mode");
 ros::ServiceClient client_operating_mode=n.serviceClient<common_msgs_gl::SendIntArray>("set_operating_mode");
 common_msgs_gl::SendIntArray srv;
 int values[size];

 for(int i=0;i<size;i++)
 {
  values[i]=modes[i];
 }
 srv.request.data ={values[0],values[1]};//Need to make this dynamic
 if(client_operating_mode.call(srv))
 {
   
   ROS_INFO("Set_operating_mode_Success");
   return 1;
 }
 else
 {
  ROS_INFO("Failure");
  return -1;
 }
}

bool command_position(ros::NodeHandle n,int motor_num, float position)
{
  //ros::NodeHandle n;
  ros::service::waitForService("cmd_pos_ind");
  ros::ServiceClient client_position=n.serviceClient<common_msgs_gl::SendDoubleArray>("cmd_pos_ind");
  common_msgs_gl::SendDoubleArray srv;
  srv.request.data ={motor_num,position};
  
  if(client_position.call(srv))
  {
   ros::Duration(0.5).sleep(); 
   ROS_INFO("Command_position_Success");
   return 1;
  }
  else
  {
  ROS_INFO("Failure");
  return -1;
 }
}

bool command_torque(ros::NodeHandle n,int motor_num, float torque)
{
  //ros::NodeHandle n;
  ros::service::waitForService("cmd_torque_ind");
  ros::ServiceClient client_torque=n.serviceClient<common_msgs_gl::SendDoubleArray>("cmd_torque_ind");
  common_msgs_gl::SendDoubleArray srv;
  srv.request.data ={motor_num,torque};
  if(client_torque.call(srv))
  {
   //ros::Duration(0.5).sleep();
   ROS_INFO("Command_torque_Success");
   return 1;
  }
  else
  {
  ROS_INFO("Failure");
  return -1;
 }
}

bool set_friction_left(ros::NodeHandle n,bool friction_surface)
{
  ros::service::waitForService("Friction_surface_Left");
  ros::ServiceClient client_torque=n.serviceClient<common_msgs_gl::SendBool>("Friction_surface_Left");
  common_msgs_gl::SendBool srv;
  srv.request.data =friction_surface;
  if(client_torque.call(srv))
  {
   //ros::Duration(0.5).sleep();
   ROS_INFO("Friction_surface_set_Success");
   return 1;
  }
  else
  {
  ROS_INFO("Failure");
  return 0;
 }
}

bool set_friction_right(ros::NodeHandle n,bool friction_surface)
{
  ros::service::waitForService("Friction_surface_Right");
  ros::ServiceClient client_torque=n.serviceClient<common_msgs_gl::SendBool>("Friction_surface_Right");
  common_msgs_gl::SendBool srv;
  srv.request.data =friction_surface;
  if(client_torque.call(srv))
  {
   //ros::Duration(0.5).sleep();
   ROS_INFO("Friction_surface_set_Success");
   return 1;
  }
  else
  {
  ROS_INFO("Failure");
  return 0;
 }
}
 

bool hold_object(ros::NodeHandle n,float position_left,float position_right)
{
 bool send_pos1,send_pos2;
 int modes1[] = {3,3};
 set_modes=set_actuator_modes(n,2,modes1);
 send_pos1=command_position(n,0,position_left);
 send_pos2=command_position(n,1,position_right); 
 ros::Duration(1).sleep();
 if(send_pos1 && send_pos2)
 {
  return true;
  }
 else
 {
  return false;
  }
}

bool slide_right_up(ros::NodeHandle n, float position)
{
 
 // To Hold the object 
 //send_pos=hold_object(n,0.52,0.84);
 
 // Setting modes [Left -Position, Right- Torque]
 int modes[]={3,0};
 set_modes=set_actuator_modes(n,2,modes);
 
 // Setting Friction Surface [Right -Low, Right- High]
 set_friction_l = set_friction_right(n,true);
 set_friction_r = set_friction_left(n,false);
 ros::Duration(1).sleep();
 
 // Execute the slide right up functionality
 if(set_friction_l && set_friction_r)
 {
  send_torque=command_torque(n,1,0.15);
  if(send_torque)
  {
    // Commanding position in increments for smooth control
    for(float i= 0.52;i>=position;i=i-0.01)
   {   
    send_pos=command_position(n,0,i);
    }
   if(send_pos)
   {
     return 1;
   	}
    else
    {
    ROS_ERROR("Sending Position values failed");
    return 0;
    }
   }
   else
   {
    ROS_ERROR("Setting torque values failed");
    return 0;  
   }
   }
  else
  {
   ROS_ERROR("Friction surface are not set");
   return 0;
  }
}

bool slide_right_down(ros::NodeHandle n, float position)
{
 
 // To Hold the object 
 //send_pos=hold_object(n,0.52,0.84);

 // Setting modes [Left -Torque, Right- Position]
 int modes[]={0,3};
 set_modes=set_actuator_modes(n,2,modes);
 
 // Setting Friction Surface [Right -Low, Right- High]
 set_friction_l = set_friction_right(n,true);
 set_friction_r = set_friction_left(n,false);
 ros::Duration(1).sleep();
 
 // Execute the slide right down functionality
 if(set_friction_l && set_friction_r)
 {
  send_torque=command_torque(n,0,0.15);
  if(send_torque)
  {
    // Commanding position in increments for smooth control
    for(float i= 0.84;i>=position;i=i-0.01)
   {   
    send_pos=command_position(n,1,i);
    }
   if(send_pos)
   {
     return 1;
   	}
    else
    {
    ROS_ERROR("Sending Position values failed");
    return 0;
    }
   }
   else
   {
    ROS_ERROR("Setting torque values failed");
    return 0;  
   }
   }
  else
  {
   ROS_ERROR("Friction surface are not set");
   return 0;
  }
}

bool slide_left_down(ros::NodeHandle n, float position)
{
 
// To Hold the object 
 //send_pos=hold_object(n,0.52,0.84);
 
 // Setting modes [Left -Position, Right- Torque]
 int modes[]={3,0};
 set_modes=set_actuator_modes(n,2,modes);
 
 // Setting Friction Surface [Right -High, Right- Low]
 set_friction_l = set_friction_right(n,false);
 set_friction_r = set_friction_left(n,true);
 ros::Duration(1).sleep();
 
 // Execute the slide left down functionality
 if(set_friction_l && set_friction_r)
 {
  send_torque=command_torque(n,1,0.15);
  if(send_torque)
  {
    // Commanding position in increments for smooth control
    for(float i= 0.52;i>=position;i=i-0.01)
   {   
    send_pos=command_position(n,0,i);
    }
   if(send_pos)
   {
     return 1;
   	}
    else
    {
    ROS_ERROR("Sending Position values failed");
    return 0;
    }
   }
   else
   {
    ROS_ERROR("Setting torque values failed");
    return 0;  
   }
   }
  else
  {
   ROS_ERROR("Friction surface are not set");
   return 0;
  }
}


bool slide_left_up(ros::NodeHandle n, float position)
{
 
 // To Hold the object 
 //send_pos=hold_object(n,0.52,0.84);
 
 // Setting modes [Left -Torque, Right- Position]
 int modes[]={0,3};
 set_modes=set_actuator_modes(n,2,modes);
 
 // Setting Friction Surface [Right -High, Right- Low]
 set_friction_l = set_friction_right(n,false);
 set_friction_r = set_friction_left(n,true);
 ros::Duration(1).sleep();
 
 // Execute the slide left up functionality
 if(set_friction_l && set_friction_r)
 {
  send_torque=command_torque(n,0,0.15);
  if(send_torque)
  {
    // Commanding position in increments for smooth control
    for(float i= 0.84;i>=position;i=i-0.01)
   {   
    send_pos=command_position(n,1,i);
    }
   if(send_pos)
   {
     return 1;
   	}
    else
    {
    ROS_ERROR("Sending Position values failed");
    return 0;
    }
   }
   else
   {
    ROS_ERROR("Setting torque values failed");
    return 0;  
   }
   }
  else
  {
   ROS_ERROR("Friction surface are not set");
   return 0;
  }
}



bool rotate_clockwise(ros::NodeHandle n,float position)
{
 // To Hold the object 
 //send_pos=hold_object(n,0.52,0.84);
 
// Setting modes [Left -Position, Right- Torque]
 int modes[]={3,0};
 set_modes=set_actuator_modes(n,2,modes);

// Setting Friction Surface [Right -High, Right- High]
 set_friction_l = set_friction_left(n,false);
 set_friction_r = set_friction_right(n,false);
 ros::Duration(1).sleep();
 
// Execute the Rotate clockwise functionality
 if(set_friction_l && set_friction_r)
 {
  send_torque=command_torque(n,1,0.15);
  if(send_torque)
  {
   ROS_INFO("TT");
   // Commanding position in increments for smooth control
    for(float i= 0.593;i>=position;i=i-0.01)
   {   
    send_pos=command_position(n,0,i);
    }
  
   if(send_pos)
   {
    ROS_INFO("SS");
    return 1;
   	}
    else
    {
    return 0;
    }
   }
   else
   {
    return 0;  
   }
   }
  else
  {
   return 0;
  }
}

// Need to change this function to get two arguments as input [start,end]
bool rotate_anticlockwise(ros::NodeHandle n,float position)
{
 // To Hold the object 
 //send_pos=hold_object(n,0.52,0.84);
 
// Setting modes [Left -Torque, Right- Position]
 int modes[]={0,3};
 set_modes=set_actuator_modes(n,2,modes);

// Setting Friction Surface [Right -High, Right- High]
 set_friction_l = set_friction_left(n,false);
 set_friction_r = set_friction_right(n,false);
 //ros::Duration(1).sleep();
 
// Execute the Rotate anticlockwise functionality
 if(set_friction_l && set_friction_r)
 {
  send_torque=command_torque(n,0,0.15);
  if(send_torque)
  {
   ROS_INFO("TT");
   // Commanding position in increments for smooth control
    for(float i= 0.93;i>=position;i=i-0.01)
   {   
    send_pos=command_position(n,1,i);
    }
  
   if(send_pos)
   {
    ROS_INFO("SS");
    return 1;
   	}
    else
    {
    return 0;
    }
   }
   else
   {
    return 0;  
   }
   }
  else
  {
   return 0;
  }
}

bool Home_position(ros::NodeHandle n)
{
 int modes1[] = {3,3};
 set_modes=set_actuator_modes(n,2,modes1);
 send_pos=command_position(n,0,0.36);
 send_pos=command_position(n,1,0.70); 
 ros::Duration(1).sleep();
}


/*int main(int argc, char *argv[])
{
 ros::init(argc,argv,"Client_controller");
 ros::NodeHandle n;
 //bool set_modes,send_pos,send_torque,read_pos;
 bool f;
 //f=slide_right_up(n,0.45);
  //f=slide_left_down(n,0.35);
  //f=slide_right_down(n,0.7845);
  //f=slide_left_up(n,0.7045);  
   //f= rotate_clockwise(n,0.35);   
   f= rotate_anticlockwise(n,0.70);



 //int modes[]={3,3};
 //set_modes=set_actuator_modes(n,2,modes);
 
 //while(read_position(n,1)>=(0.37))
 //{
 //send_pos=command_position(n,0,0.35);
 //}
 //while(read_position(n,1)<=(0.6))
 //{
 //ros::Duration(0.5).sleep();
 //send_pos=command_position(n,0,0.62);
 //}
 
/* for(int j=0;j<10;j++)
{
 set_modes=set_actuator_modes(n,2,modes);
 send_torque=command_torque(n,0,0.01);

 while(read_position(n,2)<=(0.078-0.02))
{
 ROS_INFO("yyy");
 ROS_INFO("%f",read_position(n,2));
 send_pos=command_position(n,1,0.8); 
 } 
 read_pos = read_position(n,2);

 int modes1[]={3,0};
 set_modes=set_actuator_modes(n,2,modes1);
 send_torque=command_torque(n,1,0.01);
 while(read_position(n,1)<=(0.53-0.02))
{
 ROS_INFO("xxx");
 ROS_INFO("%f",read_position(n,1));
 send_pos=command_position(n,0,0.65); 
}*/
 //ros::spin();
 //}
 //return 0;
//}

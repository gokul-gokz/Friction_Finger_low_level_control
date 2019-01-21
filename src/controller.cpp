#include <friction_finger_gripper/finger.hpp>

class finger_controller
{
 private:
	//Creating a pointer for fingers as the number of fingers are dynamic
	finger *fptr;
	DynamixelNode *nptr;
 public:
 	finger_controller(int num_fingers,float parameters[]);
};

//Initializing the fingers and its parameters [ID,length,width,friction_coefficient]
finger_controller::finger_controller(int num_fingers, float parameters[])
{
 
 std::vector<int> n;

//Note:Need to add a condition for checking whether the input has correct number of parameters
 
 for (int i =0;i<num_fingers;i++)
 {
   n.push_back(parameters[(4*i)]);
   fptr= new finger(parameters[(4*i)+1],parameters[(3*i)+2],parameters[(3*i)+3]);
  }
  DynamixelNode D("XM",n);
  ros::spin ();
  //ros::AsyncSpinner spinner(4); // Use 4 threads
  //spinner.start();
  //ros::waitForShutdown();
  
}  



int main(int argc, char **argv)
{
 ros::init(argc, argv, "finger_controller");
 float parameters[]={21,2,3,4,20,6,7,8};
 finger_controller h1(2,parameters);
 return 0;
}


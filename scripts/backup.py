
from mpmath  import *
from sympy import *
import numpy as np
import rospy
from friction_finger_gripper.srv import*
import std_msgs.msg


#flag -0 (left)
#flag -1 (right)
def angle_conversion(angle,flag): 
	if(flag==1):
		n_angle = 0.377+(139-angle*180/pi)/360
	else:
		n_angle= (angle*180/pi-41.47)/360 + 0.702
	print("n_angle",n_angle)
	return (n_angle)
		


def ik_finger(x, y, wp, w0):
	d1, t1 = symbols('d1 t2')

	# Equation for X, Y coordinates
	eqn1 = (d1 - w0/2)* cos(t1) + w0* sin(t1)/2
	eqn2 = (d1 - w0/2)* sin(t1) - w0* cos(t1)/2

	eqn3 = simplify(x**2 + y**2 - eqn1**2 - eqn2**2)
	# Solving
	sold1 = solve(eqn3, d1)
	solt1 =  solve(eqn1.subs(d1,sold1[1])-x,t1)
	 
	# Corresponding t2, d2 values
	d1v = np.array([sold1[1]*cos(solt1[1]), sold1[1]*sin(solt1[1])])
	w0v = np.array([w0*sin(solt1[1]), -w0*cos(solt1[1])])
	wpv = np.array([wp, 0])
	d2v = d1v + w0v - wpv
	d2v = np.array([float(d2v[0]), float(d2v[1])])

	t2 = np.arctan2(float(d2v[1]),float(d2v[0]))
	d2 = np.sqrt((d2v*d2v).sum())
	return (solt1[1], t2, sold1[1], d2)

def finger_translation1(w0, wp, t1, d1):

	#X, Y Coordinates of the square
	x_square = d1* cos(t1) - w0* cos(t1)/2 + w0* sin(t1)/2
	y_square = d1* sin(t1) - w0* sin(t1)/2 - w0* cos(t1)/2
	#print("x=",x_square)
	#print("y=",y_square)
	# Calculate d2, theta2
	d1v = np.array([(d1)* cos(t1), (d1)* sin(t1)] )
	w0v = np.array([w0* sin(t1), -w0* cos(t1)])
	wpv = np.array([wp, 0])
	d2v = d1v + w0v - wpv
	d2v = np.array([float(d2v[0]), float(d2v[1])])
	t2 = np.arctan2(float(d2v[1]),float(d2v[0]))
	d2 = np.sqrt((d2v*d2v).sum())
	return(t2, d2)

def finger_translation2(w0, wp, t2, d2):

	# X, Y coordinates of the square
	x_square = wp + d2*cos(t2) - w0*cos(t2)/2 - w0*sin(t2)/2
	y_square = d2* sin(t2) - w0*sin(t2)/2 + w0*cos(t2)/2
	#print("x=",x_square)
	#print("y=",y_square)
	# Calculate d1, theta1
	d2v = np.array([(d2)* cos(t2), (d2)* sin(t2)])
	w0v = np.array([w0* sin(t2), -w0* cos(t2)])
	wpv = np.array([wp, 0])	
	d1v = d2v - w0v + wpv
	d1v = np.array([float(d1v[0]), float(d1v[1])])
	t1 = np.arctan2(d1v[1], d1v[0])
	d1 = np.sqrt((d1v*d1v).sum())
	return (t1, d1)

def slide_left_finger_down(p):
	#Call Left_Slide_Down(till left most position) Assume t1 = pi/6
	rospy.wait_for_service('Slide_Left_Finger_Down')
	try:
		slide_left_down = rospy.ServiceProxy('Slide_Left_Finger_Down',PositionCommand)
		resp1 = slide_left_down(p)
	except rospy.ServiceException, e:
		print ("Service call failed: %s"%e)

def slide_left_finger_up(p):
	rospy.wait_for_service('Slide_Left_Finger_Up')
	try:
		slide_left_up = rospy.ServiceProxy('Slide_Left_Finger_Up',PositionCommand)
		resp1 = slide_left_up(p)
	except rospy.ServiceException, e:
		print ("Service call failed: %s"%e)

def slide_right_finger_down(p):
	#Call Left_Slide_Down(till left most position) Assume t1 = pi/6
	rospy.wait_for_service('Slide_Right_Finger_Down')
	try:
		slide_right_down = rospy.ServiceProxy('Slide_Right_Finger_Down',PositionCommand)
		resp1 = slide_right_down(p)
	except rospy.ServiceException, e:
		print ("Service call failed: %s"%e)

def slide_right_finger_up(p):
	rospy.wait_for_service('Slide_Right_Finger_Up')
	try:
		slide_right_up = rospy.ServiceProxy('Slide_Right_Finger_Up',PositionCommand)
		resp1 = slide_right_up(p)
	except rospy.ServiceException, e:
		print ("Service call failed: %s"%e)


def rotate_object_anticlockwise(p):
	rospy.wait_for_service('Rotate_anticlockwise')
	try:
		rotate_anti = rospy.ServiceProxy('Rotate_anticlockwise',PositionCommand)
		resp1 = rotate_anti(p)
	except rospy.ServiceException, e:
		print ("Service call failed: %s"%e)

def rotate_object_clockwise(p): #0.35
	rospy.wait_for_service('Rotate_clockwise')
	try:
		Rotate_clockwise = rospy.ServiceProxy('Rotate_clockwise',PositionCommand)
		resp1 = Rotate_clockwise(p)
	except rospy.ServiceException, e:
		print ("Service call failed: %s"%e)





def finger_planning(x, y, w0, wp, t1, t2, d1, d2):
	# Calculate Desired Position Parameters
	t1_d, t2_d, d1_d, d2_d = ik_finger(x, y, w0, wp)
	print (d1_d, float(d2_d))
	# Path Planning
	d1t = d1
	d2t = d2
	
	if (d1_d > d1 and d2_d > d2):
		while ((d1_d - d1) > 0.001 or (d2_d - d2)> 0.001):
			while (d1 < d1_d and t2 > 41.47*pi/180):
				t2 = t2 - 0.01
				t1, d1 = finger_translation2(w0, wp, t2, d2)
				#print (d1_d, d1, d2_d, d2)
			# Call Left_Slide_Up(t1, t2)
			#print (d1, d1_d, d2, d2_d)			
			#slide_left_finger_up((t2*180/pi - 23.5)*0.00218 + 0.6993)
			print("t2=",t2)
			slide_right_finger_up(angle_conversion(t2,1))
			while (d2 < d2_d and t1 < 138.53*pi/180):
				t1 = t1 + 0.01
				t2, d2 = finger_translation1(w0, wp, t1, d1)
				#print (d1_d, d1, d2_d, d2)
			# Call Right_Slide_Up(t1, t2)
			#print (d1, d1_d, d2, d2_d)			
			#slide_right_finger_up(0.4862-0.00181*(t1*180/pi-90))
			print ("t1=",t1)
			slide_left_finger_up(angle_conversion(t1,0))
			d1t = d1
			d2t = d2
			#print (d1_d, d1, d2_d, d2)
		t2, d2 = finger_translation1(w0, wp, t1_d, d1)
		# Call Left_Slide_Up(t1_d, t2)
		slide_left_finger_up(angle_conversion(t2,0))

	else:	
		while ((d1_d - d1) < 0.001 or (d2_d - d2) < 0.001):
			while (d1 > d1_d and t2 < 5*pi/6):
				t2 = t2 + 0.01
				t1, d1 = finger_translation2(w0, wp, t2, d2)
			#Call Left_Slide_Down(t1, t2)
			slide_left_finger_down(angle_conversion(t1,0))
			while (d2 > d2_d and t1 > pi/6):
				t1 = t1 - 0.01
				t2, d2 = finger_translation1(w0, wp, t1, d1)
			# Call Right_Slide_Down(t1, t2)
			slide_right_finger_down(angle_conversion(t2,1))
			d1t = d1
			d2t = d2
		t2, d2 = finger_translation1(w0, wp, t1_d, d1)
		# Call Left_Slide_Down(t1_d, t2)
		slide_left_finger_down(angle_conversion(t1_d,0))
	#print (t1_d, t1, t2_d, t2)



if __name__ == "__main__":
	#Initial position of the object in the fingers 
	d1 = 7.5
	d2 = 7.5
	#Starting angle of the fingers
	t1 = np.pi/2
	t2 = np.pi/2
	#Width of object
	w0 = 2.5
        #Width of palm
	wp = 5
	#Desired position of the object
	x=2.5
	y=15

	n = 3 # n = 1 -> Anticlockwise rotation, n = 0 -> Clockwise
	
	#Hold object
	rospy.wait_for_service('Hold_object')
	try:
		hold_object = rospy.ServiceProxy('Hold_object',Holdcommand)
		#req = friction_finger_gripper.srv.PositionCommandRequest(0.52,0.64)
		#resp1 = hold_object(req)
		resp1 = hold_object(0.50,0.85)
	except rospy.ServiceException, e:
		print ("Service call failed: %s"%e)
	
	if n == 1:	
		t1, d1 = finger_translation2(w0, wp, t2, d2)
			
		#Call Left_Slide_Down(till left most position) Assume t1 = pi/6
		slide_left_finger_down(angle_conversion(t1,0))
       
		while (t2 >= pi/6):
			t2, d2 = finger_translation1(w0, wp, t1, d1)
			t1 = t1 - 0.01
		t1, d1 = finger_translation2(w0, wp, t2, d2)
		t2f = np.arccos(((d1 - w0)**2 + w0**2 - (d2+w0)**2 - wp**2)/(2*wp*(d2 + w0))) 
		# Call Rotate_anticlockwise(t2f)

		rotate_object_anticlockwise((t2f*180/pi - 23.5)*0.00218 + 0.6993)



		d1 = d1 - w0
		d2 = d2 + w0
	if n == 0:
		t2, d2 = finger_translation1(w0, wp, t1, d1)
		#Call Right_Slide_Down(till right most position) Assume t2 = 5*pi/6
		slide_right_finger_down((t2*180/pi - 23.5)*0.00218 + 0.6993)

	
		while (t1 <= np.pi *5/6):
			t1, d1 = finger_translation2(w0, wp, t2, d2)
			t2 = t2 + 0.01
	
		t2, d2 = finger_translation1(w0, wp, t1, d1)
		t1f = np.pi - np.arccos((((d2-w0)**2 + w0**2 - wp**2 - (d1 + w0)**2)/(2*wp*(d1+w0))))
		print (t1f)


		# Call Rotate_clockwise(t1f)
		rotate_object_clockwise(0.4862-0.00181*(t1f*180/pi-90))

	finger_planning(x, y, w0, wp, t1, t2, d1, d2)	

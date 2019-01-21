from mpmath  import *
from sympy import *
import numpy as np



def ik_finger(x, y, wp, w0):
	d1, t1 = symbols('d1 t2')

	# Equation for X, Y coordinates
	eqn1 = (d1 + wp/2)* cos(t1) + wp* sin(t1)/2
	eqn2 = (d1 + wp/2)* sin(t1) - wp* cos(t1)/2

	eqn3 = simplify(x**2 + y**2 - eqn1**2 - eqn2**2)
	# Solving
	sold1 = solve(eqn3, d1)
	solt1 = solve(eqn1.subs(d1, sold1[0])-x, t1)
	# Corresponding t2, d2 values
	d1v = np.array([sold1[0]*cos(solt1[0]), sold1[0]*sin(solt1[0])])
	w0v = np.array([w0*sin(solt1[0]), -w0*cos(solt1[0])])
	wpv = np.array([wp, 0])
	d2v = d1v + w0v - wpv
	d2v = np.array([float(d2v[0]), float(d2v[1])])

	t2 = np.arctan2(float(d2v[1]),float(d2v[0]))
	d2 = np.sqrt((d2v*d2v).sum())
	return (solt1[0], t2, sold1[0], d2)

def finger_translation1(w0, wp, t1, d1):

	#X, Y Coordinates of the square
	x_square = d1* cos(t1) + w0* cos(t1)/2 + w0* sin(t1)/2
	y_square = d1* sin(t1) + w0* sin(t1)/2 - w0* cos(t1)/2

	# Calculate d2, theta2
	d1v = np.array([d1* cos(t1), d1* sin(t1)] )
	w0v = np.array([w0* sin(t1), -w0* cos(t1)])
	wpv = np.array([wp, 0])
	d2v = d1v + w0v - wpv
	d2v = np.array([float(d2v[0]), float(d2v[1])])
	t2 = np.arctan2(float(d2v[1]),float(d2v[0]))
	d2 = np.sqrt((d2v*d2v).sum())
	return(t2, d2)

def finger_translation2(w0, wp, t2, d2):

	# X, Y coordinates of the square
	x_square = wp + d2*cos(t2) + w0*cos(t2)/2 - w0*sin(t2)/2
	y_square = d2* sin(t2) + w0*sin(t2)/2 + w0*cos(t2)/2

	# Calculate d1, theta1
	d2v = np.array([d2* cos(t2), d2* sin(t2)])
	w0v = np.array([w0* sin(t2), -w0* cos(t2)])
	wpv = np.array([wp, 0])	
	d1v = d2v - w0v + wpv
	d1v = np.array([float(d1v[0]), float(d1v[1])])
	t1 = np.arctan2(d1v[1], d1v[0])
	d1 = np.sqrt((d1v*d1v).sum())
	return (t1, d1)

def finger_planning(x, y, w0, wp, t1, t2, d1, d2):
	# Calculate Desired Position Parameters
	t1_d, t2_d, d1_d, d2_d = ik_finger(x, y, w0, wp)
	print d1_d, float(d2_d)
	# Path Planning
	d1t = d1
	d2t = d2
	if (d1_d > d1 and d2_d > d2):
		while ((d1_d - d1) > 0.001 or (d2_d - d2)> 0.001):
			while (d1 < d1_d and t2 > pi/6):
				t2 = t2 - 0.01
				t1, d1 = finger_translation2(w0, wp, t2, d2)
				print d1_d, d1, d2_d, d2
			# Call Left_Slide_Up(t1, t2)
			while (d2 < d2_d and t1 < 5*pi/6):
				t1 = t1 + 0.01
				t2, d2 = finger_translation1(w0, wp, t1, d1)
				print d1_d, d1, d2_d, d2
			# Call Right_Slide_Up(t1, t2)
			d1t = d1
			d2t = d2
			print d1_d, d1, d2_d, d2
		t2, d2 = finger_translation1(w0, wp, t1_d, d1)
		# Call Left_Slide_Up(t1_d, t2)

	else:
		while ((d1_d - d1) < 0.001 or (d2_d - d2) < 0.001):
			while (d1 > d1_d and t2 < 5*pi/6):
				t2 = t2 + 0.01
				t1, d1 = finger_translation2(w0, wp, t2, d2)
			#Call Left_Slide_Down(t1, t2)
			while (d2 > d2_d and t1 > pi/6):
				t1 = t1 - 0.01
				t2, d2 = finger_translation1(w0, wp, t1, d1)
			# Call Right_Slide_Down(t1, t2)
			d1t = d1
			d2t = d2
		t2, d2 = finger_translation1(w0, wp, t1_d, d1)
		# Call Left_Slide_Down(t1_d, t2)
	print t1_d, t1, t2_d, t2


d1 = 7.5
d2 = 7.5
t1 = np.pi/2
t2 = np.pi/2
w0 = 2
wp = 2

n = 0 # n = 1 -> Anticlockwise rotation, n = 0 -> Clockwise

if n == 1:	
	t1, d1 = finger_translation2(w0, wp, t2, d2)	
	#Call Left_Slide_Down(till left most position) Assume t1 = pi/6
	while (t2 >= pi/6):
		t2, d2 = finger_translation1(w0, wp, t1, d1)
		t1 = t1 - 0.01
	t1, d1 = finger_translation2(w0, wp, t2, d2)
	t2f = np.arccos(((d1 - w0)**2 + w0**2 - (d2+w0)**2 - wp**2)/(2*wp*(d2 + w0))) 
	# Call Rotate_anticlockwise(t2f)
	d1 = d1 - w0
	d2 = d2 + w0
if n == 0:
	t2, d2 = finger_translation1(w0, wp, t1, d1)
	#Call Right_Slide_Down(till right most position) Assume t2 = 5*pi/6	
	while (t1 <= np.pi *5/6):
		t1, d1 = finger_translation2(w0, wp, t2, d2)
		t2 = t2 + 0.01
	
	t2, d2 = finger_translation1(w0, wp, t1, d1)
	t1f = np.pi - np.arccos((((d2-w0)**2 + w0**2 - wp**2 - (d1 + w0)**2)/(2*wp*(d1+w0))))
	print t1f
	# Call Rotate_clockwise(t1f)
	d1 = d1 + w0
	d2 = d2 - w0

finger_planning(x, y, w0, wp, t1, t2, d1, d2)
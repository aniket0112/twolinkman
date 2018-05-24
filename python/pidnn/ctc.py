#Computed Torque Controller
import time
import numpy as np
from collections import deque

#process variables - pv = process value & sp = setpoint, _f = front & _r = rear
angle_pv_f = deque([0,0])		#x[0] -> sample at (t-1) x[1] -> sample at (t)
angle_pv_r = deque([-90,-90])		
angle_sp_f = deque([0,0])		
angle_sp_r = deque([-90,-90])
der_angle_r = 0
der_angle_f = 0

#constants
length_r = 0
length_f = 0
mass_r = 0
mass_f = 0
g = 9.8						#gravitational acceleration
t_sample = 1
Kp = 1
Kd = 0

#time list for calculating derivatives
time_pv_f = time.time()
time_pv_r = time_pv_f
time_sp_f = time_pv_f
time_sp_r = time_pv_f

def cosd(x):
	return np.cos(np.deg2rad(x))
def sind(x):
	return np.sin(np.deg2rad(x))

# Function to update process values and setpoints
def update(front_angle_sp, front_angle_pv, rear_angle_sp, rear_angle_pv):
	global angle_sp_r, angle_sp_f, angle_pv_r, angle_pv_f
	#update setpoints
	angle_sp_f.rotate(-1)
	angle_sp_f[1] = front_angle_sp
	angle_sp_r.rotate(-1)
	angle_sp_r[1] = rear_angle_sp
	
	#update process values
	angle_pv_f.rotate(-1)
	angle_pv_f[1] = front_angle_pv	
	angle_pv_r.rotate(-1)
	angle_pv_r[1] = rear_angle_pv
	
def derivative(y,time_):
	denominator = (time.time()-time_)
	if denominator is not 0:
		return (y[1]-y[0])/denominator
	else:
		return 0

# Initial parameter setup for Computed torque control algorithm
def parameter(rear_link_length, front_link_length, rear_link_mass, front_link_mass, 
	kp, kd, initial_angle_f, initial_angle_r):
	global angle_pv_r, angle_pv_f, angle_sp_r, angle_sp_f
	global length_r,length_f,mass_r,mass_f,t_sample,Kp,Kd
	global time_pv_f, time_sp_f, time_pv_r, time_sp_r
	length_r = rear_link_length
	length_f = front_link_length
	mass_r = rear_link_mass
	mass_f = front_link_mass
	Kp = kp
	Kd = kd
	#time list for calculating derivatives
	time_pv_f = time.time()
	time_pv_r = time_pv_f
	time_sp_f = time_pv_f
	time_sp_r = time_pv_f
	#initial values
	angle_pv_f = deque([initial_angle_f,initial_angle_f])
	angle_pv_r = deque([initial_angle_r,initial_angle_r])

# Controller function. Returns torque values for given setpoint and process values.
# Always call this function after updating pv and sp by 'update' function
def controller():
	global angle_pv_r, angle_pv_f, angle_sp_r, angle_sp_f
	global length_r,length_f,mass_r,mass_f,t_sample,Kp, Kd
	global time_pv_f, time_sp_f, time_pv_r, time_sp_r

	global der_angle_r, der_angle_f

	der_angle_r = derivative(angle_pv_r, time_pv_r)
	time_pv_r = time.time()
	der_angle_f = derivative(angle_pv_f, time_pv_f)
	time_pv_f = time.time()
	desired_der_angle_r = derivative(angle_sp_r, time_sp_r)
	time_sp_r = time.time()
	desired_der_angle_f = derivative(angle_sp_f, time_sp_f)
	time_sp_f = time.time()

	angle_r = angle_pv_r[1]
	angle_f = angle_pv_f[1]
	desired_angle_r = angle_sp_r[1]
	desired_angle_f = angle_sp_f[1]
	
	D = np.matrix([[(mass_r+mass_f)*length_r**2+mass_f*length_f**2+2*mass_f*length_r*length_f*cosd(angle_f), mass_f*length_f**2+mass_f*length_r*length_f*cosd(angle_f)], [mass_f*length_f**2+mass_f*length_r*length_f*cosd(angle_f), mass_f*length_f**2]])
	H = np.matrix([[-mass_f*length_r*length_f*(2*der_angle_r*der_angle_f+der_angle_f**2)*sind(angle_f)], [mass_f*length_r*length_f*der_angle_r**2*sind(angle_f)]])
	C = np.matrix([[(mass_r+mass_f)*length_r*g*cosd(angle_r)+mass_f*length_f*g*cosd(angle_r+angle_f)], [mass_f*length_f*g*cosd(angle_r+angle_f)]])

	ANGLE = np.matrix([[angle_r],[angle_f],[der_angle_r],[der_angle_f]])
	DESIRED_ANGLE = np.matrix([[desired_angle_r],[desired_angle_f],[desired_der_angle_r],[desired_der_angle_f]])
	E = DESIRED_ANGLE - ANGLE
	if(abs(E.item(0)) < 1 and abs(E.item(1)) < 1 and abs(E.item(2)) < 1 and abs(E.item(3)) < 1):
		U = np.matrix([[0],[0]])
	else:
		U = -Kd*np.matrix([[E.item(2)],[E.item(3)]])-Kp*np.matrix([[E.item(0)],[E.item(1)]])
	T = D*(-U)+H+C
#	print(str(der_angle_r)+':'+str(der_angle_f)+':'+str(angle_r)+':'+str(angle_f)+':'+str(T))
#	print(time.time())
	return T
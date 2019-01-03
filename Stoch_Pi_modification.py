from __future__ import division
import sys
import sysv_ipc 
sys.path.append('.')
import RTIMU
import os.path
import time
from smbus import SMBus
import Adafruit_PCA9685
import math
from SherControl_Scripts_global import Inverse_Kinamatics as IK
from SherControl_Scripts_global import Jacobian_Inv as JInv
from SherControl_Scripts_global import *
import numpy as np
from non_blocking import *
from Library_stoch import *

PI = math.pi # Pi 

# Defining IMU as an input
SETTINGS_FILE = "RTIMULib"

print("Using settings file " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):
        print("Settings file does not exist, will be created")

s = RTIMU.Settings(SETTINGS_FILE)
print s
imu = RTIMU.RTIMU(s)
print imu
print("IMU Name: " + imu.IMUName())

if (not imu.IMUInit()):
        print("IMU Init Failed")
        # sys.exit(1)
else:
    print("IMU Init Succeeded")
    # this is a good time to set any fusion parameters
    imu.setSlerpPower(0.02)
    imu.setGyroEnable(True)
    imu.setAccelEnable(True)
    imu.setCompassEnable(True)

    poll_interval = imu.IMUGetPollInterval()
    print("Recommended Poll Interval: %dmS\n" % poll_interval)
# Defining functions for key board input\

def HigherControl(k, v0):
	global Phi0, Stabilize, Break, Gait
	# Oscillation control
	if k == 'w':
            v = v0 + 0.05
	elif k == 's':
            v = v0 - 0.05
	else:
            v = v0
	# Turning variable
	if k == 'a':
            omega = 1
	elif k == 'd':
            omega = -1
	else:
            omega = 0

	if k == 'b':		# Breaking the robot
		omega = 0; v = 0;
		Break = 1
	else:
                Break = 0

	if k == 'l':
                Phi0 = [PI, PI, 0, 0]
	elif k=='k':
		Phi0 =[0, 0, 0, 0]

        if k == 't':
            x1 = 'Floor'
            if Gait == x1:
                Gait = 'Carpet'
                print([Gait, x1])
            elif Gait == 'Carpet':
                Gait = 'Floor'
                
        if k == ' ' and Stabilize == 'off':
            print("Stabilization ON")
            Stabilize = 'on'
        elif k == ' ' and Stabilize == 'on':
            print("Stabilization OFF")
            Stabilize = 'off'
	return [v, omega]

global v0, Phi0, Stabilize, Break, Gait
v0 = 0.0
Phi0 = [0, 0, 0, 0]
kb = KBHit() # creating Object of class KBHit
# Get key board value funtion

def get_input(k):
	global v0
	[v0, omega] = HigherControl(k, v0)
	#print ("V, w" )
	#print ([v0, omega])
	return [v0, omega]

#Global Variables
pwm1 = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
pwm1.set_pwm_freq(50)
pwm2 = Adafruit_PCA9685.PCA9685(address=0x41, busnum=1)
pwm2.set_pwm_freq(50)
bus = SMBus(1)

n_FRK = 0; n_FRH = 1;  n_FRA = 2
n_FLK = 15; n_FLH = 14; n_FLA = 13
n_HRK = 15; n_HRH = 14; n_HRA = 13
n_HLK = 0; n_HLH = 1; n_HLA = 2

# Joint data
#end_data = np.loadtxt('end_effector_data_Trot_5.txt')
end_data_carpet = np.loadtxt('end_effector_data_Trot_5.txt')
end_data = np.loadtxt('end_effector_data.txt')

# Theta, percent, FLK, FLH, HLK, HLH, FRK, FRH, HRK, HRH
# 0      1          2   3   4    5      6   7   8    9

# Leg geometry
lh = -0.12 		# Hip link length considering -ve axis
lk = -0.13		# Knee link length

Lh = [0, lh]		# Vectorizing: all links are set to zero about -ve Y-axis
Lk = [0, lk]		# Vectorizing: all links are set to zero about -ve Y-axis

body_geometry = [0.35, 0.2] # Body length and width
# frequency in Hertz
f = 2.5			# frequency of oscillation in Hz
w = 2*PI*f		# Hz to sec/rad conversion

# Integration time
dt = 0.05
T_run = 20000

# Motor inputs

m_FLK = 0; m_FLH = 1; m_FLA = 2
m_FRK = 3; m_FRH = 4;  m_FRA = 5
m_HLK = 6; m_HLH = 7; m_HLA = 8
m_HRK = 9; m_HRH = 10; m_HRA = 11

PWM = [300, 300, 325, 325, 300, 300, 400, 300, 325, 325, 300, 300]

# Motor referencing
MR = [210, 195, 300, 265, 420, 460, 430, 250, 350, 315, 500, 160]
# [400, 530, 310, 265, 140, 275, 430, 380, 342, 315, 140, 160] Banckup of the last  working value
# [FLK  FLH  FLA  FRA  FRH  FRK  HLH  HLK  HLA  HRA  HRK  HRH]

MR[m_FRK] = 460; MR[m_FRH] = 225; MR[m_FRA] = 265
MR[m_FLK] = 180; MR[m_FLH] = 530; MR[m_FLA] = 300
MR[m_HRK] = 500; MR[m_HRH] = 220; MR[m_HRA] = 315
MR[m_HLK] = 265; MR[m_HLH] = 470; MR[m_HLA] = 350

Theta = end_data[:,0]
Dtheta = Theta[1] - Theta[0]
Theta_min = min(Theta)

y_floor= [end_data[:,3], end_data[:,7], end_data[:,5], end_data[:,9]]
x_floor = [end_data[:,2], end_data[:,6], end_data[:,4], end_data[:,8]]

y_carpet = [end_data_carpet[:,3], end_data_carpet[:,7], end_data_carpet[:,5], end_data_carpet[:,9]]
x_carpet = [end_data_carpet[:,2], end_data_carpet[:,6], end_data_carpet[:,4], end_data_carpet[:,8]]

y_end = y_floor
x_end = x_floor

flag = 0
clock_begin = time.time()

q = np.zeros([12, 1])
q_3d = np.zeros([12, 1])

theta = [0, 0, 0 , 0]
theta0 = [0, 0, 0 , 0]
omega = 0 # Intiating turning variable
fre_out = 0 # Initiating oscillation frequrncy (Hz)
Phi = [0, 0, 0, 0]
phi = [0, 0, 0, 0]

alpha_phi = 1.25
Break = 0
Gait = 'Floor'
Stabilize = 0
while 1:
    # Reading from IMU data
    if imu.IMURead():
	    begin_temp = time.time()
            data = imu.getIMUData()
            # reading the fused data ( Roll-Pitch-Yaw)
            fusionPose = data["fusionPose"]
	    fusionVel = data["gyro"] # reading the values from the gyroscope
	    Acc = data["accel"] # reading the data from the accelerometer
	    
    if kb.kbhit():
	c = kb.getch()
	if ord(c) == 27: # ESC
		break
	[fre_out, omega]=get_input(c)

    begin_time =  time.time()

    if flag == 0:
        Leg_no = input("Which leg do you wish to operate ?  ( FL(0) / FR(1) / HL(2) / HR(3) / All(4): \n >> ")
	flag = 1;
	T = 0

    if T == T_run:
		T = 0
    else:
		T += 1
    # Omega to motion
    if omega <0:
        Amp_turn = np.array([0, 0, 1, 1])
    elif omega > 0:
        Amp_turn = np.array([1, 1, 0, 0])
    else:
        Amp_turn = np.array([1, 1, 1, 1])

    # Inverse kinematics and angle transformation of individual legs
    x_offset = [0, 0, -0.005, -0.005]
    y_offset = [-0.0175, -0.0175, -0.0175, -0.0175]
    
    # Checking out for joint stabilization
    if Stabilize=='on':
        [leg_x, leg_y, leg_z] = body_stabilize(fusionPose, lh, lk, body_geometry)
    
    # Checking out for fall prevention
    if fall_prevention == 'on':
        [leg_x_fall, leg_y_fall, leg_z_fall] =fall_prevention(Acceleration, lh, lk, body_geometry)
        
    for i in range(4):
	Phi_var = alpha_phi*(Phi0[i]-phi[i]) * dt + phi[i]
	phi[i] = Phi_var
        theta[i] = 2*PI*fre_out * dt +  theta0[i]				# Eular integration
        theta_internal =(theta[i] + Phi_var) % (2*PI)		# mod(theta, 2 pi)
    	ip = int((theta_internal - Theta_min) / Dtheta)
    	dtheta = theta_internal - Theta[ip-1]			# difference
    	
        if Gait == 'Floor':
            y_end = y_floor
            x_end = x_floor
        elif Gait =='Carpet':
            y_end = y_carpet
            x_end = x_carpet
        # Introducing Stavilization: The stabilization code works by adding offsets to the
                                    # link lengths. These offsets can be link lengths or function of link lengths.
                                    # Majority of the addition is done in Y axis.
        if Stabilize == 'on':
            x_offset_delta = leg_x[i]
            y_offset_delta = leg_z[i]
        else:
            x_offset_delta = 0
            y_offset_delta = 0
        
        # Introducing fall prevention: The fall prevention works by changing the body orientation to account
                                    # for the undesired acceleration
        if fall_prevention == 'on':
            x_offset_delta = leg_x_fall[i]
            y_offset_delta = leg_z_fall[i]
        else:
            x_offset_delta = 0
            y_offset_delta = 0
            
	# First order approximation
	if Break == 1:  # This brakes the robot and makes the robot come to
                        # a halting pose
            x_data = 0
            y_data = -0.2
        else:
            # Need to insert the filters
            x_data = x_end[i][ip-1] + (x_end[i][ip] - x_end[i][ip-1]) * dtheta/Dtheta + x_offset[i]
            y_data = y_end[i][ip-1] +  (y_end[i][ip] - y_end[i][ip-1]) * dtheta/Dtheta + y_offset[i]
            
            #kMP[0] = cos(Theta); dkMP[0] = -sin(Theta)
            #kMP[1] = cos(2*Theta); dkMP[1] = -2*sin(2*Theta)
            #kMP[2] = sin(Theta); dkMP[2] = cos(Theta)
            #kMP[3] = sin(2*Theta); dkMP[3] = 2*cos(2*Theta) 
        
            # First order approximation
            #x_data = x_offset[0] + np.dot(np.array(kMP), np.array(W[:,2*i]))
            #y_data = y_offset[1] + np.dot(np.array(kMP), np.array(W[:,2*i+1]))

	r_desired = [-Amp_turn[i] * x_data, y_data]

        # lowpass selective filter 
        dr = dr_desired + alpha*(r_desired - r)
        r = dr * dt + r
        # 

        q_IK = IK(Lh, Lk, r)
	# Angle transformation between Robot Actuator frame and
	# calculation assumption
	# [0-Fl 1-Hl 2-Fr 3-Hr]		# 		Fl Hl Fr Hr
	q_3d[3*i] = PI/2 + q_IK[0] 	# Hip: 		0  3  6  9
        q_3d[3*i+1] = PI/2 - q_IK[1] 	# Knee: 	1  4  7  10
	q_3d[3*i+2] = 0 		# Abduction: 	2  5  8  11


    # Connection/ Transformation between input Data format and Motor input format
    # [Fl Hl Fr Hr] >>> [Fl Fr Hl Hr]
    # Fl > 0 1 2 <-----> 1 0 2  < FL
    # Hl > 3 4 5 -----> 
    # Fr > 6 7 8 ----->
    # Hr > 9 10 11 --->
    qM = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    #    [FLK,     FLH,     FLA,     FRA,     FRH,     FRK,     HLH,     HLK,     HLA,     HRA,      HRK,      HRH]
    qMotor = np.array(qM)*180/PI
    qM[m_FRK] = q_3d[7]; qM[m_FRH] = q_3d[6]; qM[m_FRA] = q_3d[8]
    qM[m_FLK] = q_3d[1]; qM[m_FLH] = q_3d[0]; qM[m_FLA] = q_3d[2]
    qM[m_HRK] = q_3d[10]; qM[m_HRH] = q_3d[9]; qM[m_HRA] = q_3d[11]
    qM[m_HLK] = q_3d[4]; qM[m_HLH] = q_3d[3]; qM[m_HLA] = q_3d[5]

    for i in range(0, 12):
	 		if i in [m_FRA, m_HLA, m_HLK, m_FLK]:  PWM[i] = int(MR[i] + qM[i] * 457 / PI)
			elif i in[m_FLA, m_HRA, m_HRK, m_FRK]: PWM[i] = int(MR[i] - qM[i] * 457 / PI)
	 		elif i in [m_FRH,m_HRH]:               PWM[i] = int(MR[i] + 2 * qM[i] * 160 / PI)
			else:           	        	PWM[i] = int(MR[i] - 2 * qM[i] * 160 / PI)

    if Leg_no == 4 or Leg_no == 3: pwm2.set_pwm(n_HRH, 0, PWM[m_HRH])
    if Leg_no == 4 or Leg_no == 3: pwm2.set_pwm(n_HRK, 0, PWM[m_HRK])
    #if Leg_no == 4 or Leg_no == 3: pwm2.set_pwm(n_HRA, 0, PWM[m_HRA])

    #if Leg_no == 4 or Leg_no == 2: pwm2.set_pwm(n_HLA, 0, PWM[m_HLA])
    if Leg_no == 4 or Leg_no == 2: pwm2.set_pwm(n_HLH, 0, PWM[m_HLH])
    if Leg_no == 4 or Leg_no == 2: pwm2.set_pwm(n_HLK, 0, PWM[m_HLK])

    if Leg_no == 4 or Leg_no == 0: pwm1.set_pwm(n_FLK, 0, PWM[m_FLK])
    if Leg_no == 4 or Leg_no == 0: pwm1.set_pwm(n_FLH, 0, PWM[m_FLH])
    #if Leg_no == 4 or Leg_no == 0: pwm1.set_pwm(n_FLA, 0, PWM[m_FLA])

    #if Leg_no == 4 or Leg_no == 1: pwm1.set_pwm(n_FRA, 0, PWM[m_FRA])
    if Leg_no == 4 or Leg_no == 1: pwm1.set_pwm(n_FRH, 0, PWM[m_FRH])
    if Leg_no == 4 or Leg_no == 1: pwm1.set_pwm(n_FRK, 0, PWM[m_FRK])

    theta0 = theta
    print [fre_out, omega, Phi0, Gait, Break, dt*1000]
    end_time = time.time()
    dt = end_time  - begin_time


kb.set_normal_term()

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
from SherControl_Scripts_global import Inverse_Kinamatics_polar as IK_polar
from SherControl_Scripts_global import Jacobian_Inv as JInv
from SherControl_Scripts_global import *
import numpy as np
from non_blocking import *
from Library_stoch import *

PI = math.pi # Pi 

# Defining functions for key board input\

def HigherControl(k, v0):
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

	if k == 'b':
		omega = 0; v = 0

	return [v, omega]

global v0
v0 = 0.0
kb = KBHit() # creating Object of class KBHit
# Get key board value funtion

def get_input(k):
	global v0
	[v0, omega] = HigherControl(k, v0)
	print ("V, w")
	print ([v0, omega])
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

# Theta, percent, FLK, FLH, HLK, HLH, FRK, FRH, HRK, HRH
# 0      1          2   3   4    5      6   7   8    9

# Leg geometry
lh = -0.12 		# Hip link length considering -ve axis
lk = -0.13		# Knee link length

Lh = [0, lh]		# Vectorizing: all links are set to zero about -ve Y-axis
Lk = [0, lk]		# Vectorizing: all links are set to zero about -ve Y-axis
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
MR[m_HRK] = 500; MR[m_HRH] = 190; MR[m_HRA] = 315
MR[m_HLK] = 250; MR[m_HLH] = 470; MR[m_HLA] = 350


flag = 0
clock_begin = time.time()

q = np.zeros([12, 1])
q_3d = np.zeros([12, 1])

theta = 0
theta0 = 0
omega = 0 # Intiating turning variable
fre_out = 0 # Initiating oscillation frequrncy (Hz)

W= np.loadtxt('Weight_RL_CS.txt').transpose()
Phi = [0, PI, PI, 0]
# [-0.0503245,  -0.06732678,  1.0, -0.03611052,  1.0, -0.11518157, -0.11246687, -0.04629093,  0.12832975, -0.06655705,  1.0,  -0.15476286,  1.0, 0.0516125,   0.0841161,   0.18423271]
r_off = [0.175, 0]
while 1:
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
    print [fre_out, omega]
    # Omega to motion
    if omega <0:
        Amp_turn = np.array([0, 0, 1, 1])
    elif omega > 0:
        Amp_turn = np.array([1, 1, 0, 0])
    else:
        Amp_turn = np.array([1, 1, 1, 1])
        
    # Oscillation phase
    theta = (2*PI*fre_out * dt +  theta0)%(2*PI)				# Eular integration
    kMP = [0,0,0,0]

    # Inverse kinematics and angle transformation of individual legs
    for i in range(4):
	Theta = (theta + Phi[i])% (2*PI)
	# Determining the kMP
	kMP[0] = cos(Theta)
	kMP[1] = cos(2*Theta)
	kMP[2] = sin(Theta)
	kMP[3] = sin(2*Theta)

	# First order approximation
        l_data = r_off[0] + np.dot(np.array(kMP), np.array(W[:,2*i])) * 0.015
        theta_data = r_off[1] + 8*  np.dot(np.array(kMP), np.array(W[:,2*i+1])) *PI/180.

	r = [l_data, Amp_turn[i]*theta_data]
	q_IK = IK_polar(Lh, Lk, r)
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
    end_time = time.time()
    dt = end_time  - begin_time


kb.set_normal_term()


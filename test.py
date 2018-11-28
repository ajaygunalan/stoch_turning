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

def HigherControl(k, v0):

	if k == 'w':
            v = v0 + 0.001

	elif k == 's':
            v = v0 - 0.001

	else:
            v = v0

	if k == 'a':
            omega = 0.25

	elif k == 'd':
            omega = -0.25

	else:
            omega = 0

	return [v, omega]


global v0
v0 = 0.0


def get_input(k):
	global v0
	[v0, omega] = HigherControl(k, v0)
	print ("V, w")
	print ([v0, omega])


kb = KBHit()
PI = math.pi

#Global Variables
pwm1 = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
pwm1.set_pwm_freq(50)
pwm2 = Adafruit_PCA9685.PCA9685(address=0x41, busnum=1)
pwm2.set_pwm_freq(50)
bus = SMBus(1)


#Create a key
key = 12

# Create shared memory object
#memory = sysv_ipc.SharedMemory(key)

def read_encoder():
        memory_value = memory.read()

        #Putting those values in a lsit
        encoder_str = []
        encoder_str = memory_value.split('\0')
        encoder_str = filter(None , encoder_str)

        #Convert str into list of integr
        encoder_val = []
        encoder_val = map(int, encoder_str)
        en_ordered=[]
        for en in (0,len(encoder_val)):
              en_ordered.append((encoder_val[0]))
              en_ordered.append((encoder_val[1]))
              en_ordered.append((encoder_val[2]))
              en_ordered.append((encoder_val[3]))
              en_ordered.append((encoder_val[4]))
              en_ordered.append((encoder_val[5]))
              en_ordered.append((encoder_val[6]))
              en_ordered.append((encoder_val[7]))
              en_ordered.append((encoder_val[8]))
              en_ordered.append((encoder_val[9]))
              en_ordered.append((encoder_val[10]))
              en_ordered.append((encoder_val[11]))
        #print en_ordered
        return en_ordered

PI = math.pi
# =================================== Joint data ============================================
file_name = 'Hind'
# end_data = np.loadtxt('end_effector_data_'+ file_name + '.txt')
# Theta, percent, FLK, FLH, HLK, HLH, FRK, FRH, HRK, HRH
# 0      1          2   3   4    5      6   7   8    9

# Leg geometry
lh = -0.12 		# Hip link length considering -ve axis
lk = -0.12		# Knee link length

Lh = [0, lh]		# Vectorizing: all links are set to zero about -ve Y-axis
Lk = [0, lk]		# Vectorizing: all links are set to zero about -ve Y-axis

# frequency in Hertz
fre = 0	# frequency of oscillation in Hz
w = 2*PI*fre		# Hz to sec/rad conversion


q = np.zeros([14, 1])
q_3d = np.zeros([14, 1])

theta = [0 , 0, 0 , 0]
theta0 = [0 , 0, 0 , 0]

i_count = 1

while True:

    if kb.kbhit():
       c = kb.getch()
       if ord(c) == 27: # ESC
          break
       [fre_out, omega]=get_input(c)

    #omega = 0
    fre = LowPassFilter(fre, dt, 0, fre_out, 1.25) # need to print  frequency
    w = 2*PI*fre # convert in to radians

    # Inverse kinematics and angle transformation of individual legs
    r_rec = [0, 0, 0, 0, 0, 0, 0, 0]

    # Omega to motion
    if omega <0:
        Amp_turn = np.array([-0.5, -0.5, 1, 1])
    elif omega > 0:
        Amp_turn = np.array([1, 1, -1, -1])
    else:
        Amp_turn = np.array([1, 1, 1, 1])

    for i in range(4):
         theta[i] = w * dt +  theta0[i]				# Eular integration
         theta_internal =(theta[i] + Phase[i]) % (2*PI)		        # mod(theta, 2 pi)
         ip = int((theta_internal - Theta_min) / Dtheta)
         dtheta = theta_internal - Theta[ip-1]
	 # First order approximation
         x_ep = x_end[ip-1] +  (x_end[ip] - x_end[ip-1]) * dtheta/Dtheta
         y_ep = y_end[ip-1] +  (y_end[ip] - y_end[ip-1]) * dtheta/Dtheta

        # Offset control of the legs
         if i in [1,3]:		off_y = -0.15
         else:		off_y = -0.175

	 # Turning stategy
         x_data = Amp_turn[i]*x_ep
         y_data = y_ep

         # combining actions
         r = [0.02*fre*x_data, 0.01*fre*y_data + off_y]
         #r = [0, -0.175+0.35*fre]
	  # Inverse kinematics
         q_IK = Inverse_Kinamatics(Lh, Lk, r)

         r_rec[2*i] = r[0]
         r_rec[2*i + 1] = r[1]

         # Angle transformation between Robot Actuator frame and
         # calculation assumption
         # [0-Fl 1-Hl 2-Fr 3-Hr]		      # 		Fl Hl Fr Hr
         q_3d[3*i] = q_IK[0] * 1	      # Hip: 		0  3  6  9
         q_3d[3*i+1] = -q_IK[1] * 1	      # Knee: 	        1  4  7  10
         q_3d[3*i+2] = 0 		      # Abduction: 	2  5  8  11
    q_joint = np.array(q_3d)
    Knee = q_joint[[1, 7, 4, 10]];
    Hip = q_joint[[0, 6, 3, 9]];
 #    #Abduction = q_joint[[2, 8, 5, 11]]
 #    #Spine = q_joint[[12, 13]]

    # ====================== Out put motor ==========================================
    for i in range(4):
        #m1[1].setPosition(Abduction[i])
        m_H[i].setPosition(Hip[i][0])
        m_K[i].setPosition(Knee[i][0])
    #ms[0].setPosition(Spine[0])
    #ms[1].setPosition(Spine[2])
    print([fre_out, omega,theta_internal, r])

    # -----------------------------------------------------------
    theta0 = theta
    i_count += 1

kb.set_normal_term()

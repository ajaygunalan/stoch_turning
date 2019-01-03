from SherControl_Scripts_global import *
import math  
# Roll, pitch, yaw
# 
leg_x, leg_y, leg_z = body_stabilize([math.pi * 0, math.pi * 0.15, 0], [1, 1])

print [leg_x, leg_y, leg_z]
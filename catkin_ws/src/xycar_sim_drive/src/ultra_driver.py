#!/usr/bin/env python

import rospy, math
from std_msgs.msg import Int32MultiArray

# Data Store Order (_ is discarded)
# (front left, front middle, fornt right, _, _, _, right, left)
fl, fm, fr, r, l = 0, 0, 0, 0, 0
before_data = (0, 0, 0, 0, 0)

angle = 0
speed = 10
def callback(msg):
    global fl, fm, fr, r, l     # Use global variable fl, fm, fr, r, l
    fl, fm, fr, _, _, _, r, l = msg.data
    # Without global keyword, assign the value at local variables fl, fm, fr, r, l
    print(msg.data)

rospy.init_node('guide')
motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)
ultra_sub = rospy.Subscriber('ultrasonic', Int32MultiArray, callback)

xycar_msg = Int32MultiArray()

while not rospy.is_shutdown():
    # ===== Code Write ===== #
    if fm > 400: # If there is no obstacle at front middle, speed up
        speed = 30
    elif fm > 200: # Obstacle distance over 200 and less than 400, speed down
        speed = 15
    else: # Obstacle close, speed down
        speed = 10
    if fr > 200:
        angle = 20
    else:
        angle = 0
    # ===== ======== ===== #
    # xycar data set, [angle, speed]
    xycar_msg.data = [angle, speed]
    motor_pub.publish(xycar_msg)
    
# If you want to run code, follow below
# Open Terminal(ctrl + alt + T) at VMWare 
# Cheange Directory to ~/catkin_ws (cd ~/catkin_ws)
# build code (cm) (Once you've done it, you don't have to do it any more.)
# run code(roslaunch xycar_sim_drive xycar_sim_drive.launch) Press tap key(Auto-complete)

# Modify ====== Code Write ====== Part 
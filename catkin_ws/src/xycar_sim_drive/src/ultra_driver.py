#!/usr/bin/env python

import rospy, math
from std_msgs.msg import Int32MultiArray

fl, fm, fr, r, l = 0, 0, 0, 0, 0
before_data = (0, 0, 0, 0, 0)

angle = 0
speed = 50
def callback(msg):
    global fl, fm, fr, r, l
    fl, fm, fr, _, _, _, r, l =msg.data
    print(msg.data)

rospy.init_node('guide')
motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)
rospy.Subscriber('ultrasonic', Int32MultiArray, callback)

xycar_msg = Int32MultiArray()

while not rospy.is_shutdown():

    if abs(fr-fl) < 5:
       angle = 0
    else: 
       angle = (fr-fl)*10

    xycar_msg.data = [angle, speed]
    motor_pub.publish(xycar_msg)

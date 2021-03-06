#! /usr/bin/env python

import rospy, math
import cv2, time, rospy
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers

from tf.transformations import euler_from_quaternion

from std_msgs.msg import Int32MultiArray

arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
offset = 200
default_speed = 20

speed, roll, pitch, yaw = default_speed, 0, 0, 0
x, y = 0, 0
theta = 0
angle, distance = 0, 1000
btime, ctime = 0, 0
go_back_flag = False


def callback(msg):
    global arData, ctime, speed, theta, go_back_flag

    if len(msg.markers) == 0:
        speed = 10
        go_back_flag = True
        pass

    for i in msg.markers:
        ctime = i.header.stamp.secs
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z

        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w
    try:
        theta = math.degrees(math.atan(arData['DX'] / arData['DY']))
    except Exception as e:
        theta = 0

rospy.init_node('ar_drive')

rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)

motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size =1 )

xycar_msg = Int32MultiArray()


while not rospy.is_shutdown():
    if go_back_flag:
        print('go back flag!')
        stime = btime
        speed = -default_speed
        angle = 0
        xycar_msg.data = [angle, speed]
        motor_pub.publish(xycar_msg)
        if ctime - stime > 2:
            go_back_flag = False
            speed = default_speed
            angle = 0

    else:        
        (roll,pitch,yaw)=euler_from_quaternion((arData["AX"],arData["AY"],arData["AZ"], arData["AW"]))
        
        roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        yaw = math.degrees(yaw)

        x = arData['DX'] - offset * math.sin(math.radians(theta + yaw))
        y = arData['DY'] - offset * math.cos(math.radians(theta + yaw))

        img = np.zeros((100, 500, 3))

        img = cv2.line(img,(25,65),(475,65),(0,0,255),2)
        img = cv2.line(img,(25,40),(25,90),(0,0,255),3)
        img = cv2.line(img,(250,40),(250,90),(0,0,255),3)
        img = cv2.line(img,(475,40),(475,90),(0,0,255),3)

        point = int(arData["DX"]) + 250

        if point > 475:
            point = 475

        elif point < 25 : 
            point = 25	

        img = cv2.circle(img,(point,65),15,(0,255,0),-1)  
    
        distance = math.sqrt(pow(arData["DX"],2) + pow(arData["DY"],2))
        
        cv2.putText(img, str(int(distance))+" pixel", (350,25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255))

        dx_dy_yaw = "DX:"+str(int(arData["DX"]))+" DY:"+str(int(arData["DY"])) \
                    +" Yaw:"+ str(round(yaw,1)) 
        cv2.putText(img, dx_dy_yaw, (20,25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255))

        cv2.imshow('AR Tag Position', img)
        cv2.waitKey(1)

        if distance != 0:
            try:
                if abs(x) > 10 and abs(y) > 10 and distance > 250:
                    ang = math.degrees(math.atan(x / y)) 
                else:
                    direction = arData['DX'] // abs(arData['DX'])
                    ang = 20 * direction
                angle = ang if ang < 50 else 50
            except ValueError as e:
                print('arData: ' + str(arData['DX']) + 'distance: ' + str(distance))
        if arData["DX"] <= 60 and arData["DY"] <= 80 and distance <= 70:
            speed = 0
        else:
            speed = default_speed
        xycar_msg.data = [angle, speed]
        motor_pub.publish(xycar_msg)

        if btime != ctime:
            btime = ctime
            values = {'time': ctime, "DX": arData['DX'], "DY": arData['DY'], 'yaw': yaw,
                'distance': distance, 'angle': angle, 'speed': speed, 'x': x, 'y': y}
            print(values)

cv2.destroyAllWindows()



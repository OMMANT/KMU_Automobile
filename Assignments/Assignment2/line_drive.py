#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2, random, math, time

Width = 640
Height = 480
Offset = 330

div_backup = 0

line1_backup = 0
line2_backup = 0
angle_backup = 0

roi = np.array([[(0,390), (0, 340), (640, 390), (640, 340)]], dtype = np.int32)

def set_roi(img, roi, color3 = (255, 255, 255), color1 = 255): 
	mask = np.zeros_like(img) 
	if len(img.shape) > 2:
		color = color3
	else: 
		color = color1
	cv2.fillPoly(mask, roi, color)
	ROI_image = cv2.bitwise_and(img, mask)
	return ROI_image


def HoughLines(image, mid_y_roi):
	lines1 = cv2.HoughLines(image, 1, np.pi / 180, 20, 100, 0, 0, 100 * np.pi / 180, np.pi)
	lines2 = cv2.HoughLines(image, 1, np.pi / 180, 20, 100, 0, 0, 0, 80 * np.pi / 180)
	result = []

	if lines1 is None and lines2 is None:
		print("Can't find line")
		return result

	if lines2 is None:
		res = lines1
	elif lines1 is None:
		res = lines2
	else: 
		res = np.vstack((lines2, lines1))

	for line in res:
		rho, theta = line[0]
		cos, sin, tan = np.cos(theta), np.sin(theta), np.tan(theta)
		x1, y1 = int(cos * rho + tan * (sin * rho - mid_y_roi)), int(mid_y_roi)
		x2, y2 = int(x1 + mid_y_roi * tan), 0
		if y2 > y1:
			y2 = -y2
			x2 = -x2
		result.append([x1, y1, x2, y2])

	return result


def find_main_line(linearray, binding_pixel):
	linearray.sort(key = lambda x : x[0])
	mainline = []
	s = [0, 0, 0, 0]
	if len(linearray) > 0:
		for j in range(4): 
			s[j] = (s[j] + linearray[0][j])
		count = 1
		tic = 0
	
		for i in range(len(linearray) - 1):        
			if (linearray[i + 1][0]- linearray[i][0]) > binding_pixel:
				for j in range(4): 
					s[j] = s[j] // count
				mainline.append(s)          
				s = [0, 0, 0, 0]
				for j in range(4): 
					s[j] = (s[j] + linearray[i + 1][j])
				count = 1
				tic = 1
			else:
				for j in range(4): 
					s[j] = (s[j] + linearray[i + 1][j])
				count = count + 1
				tic = 0
		if tic == 0:
			for j in range(4): 
				s[j] = s[j] // count
			mainline.append(s)
		return mainline
	else: 
		print ("Can't find line") 
		return mainline


def find_intersection_point(line1, line2):
	global div_backup
	
	xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
	ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1]) 

	def det(a, b):
		return a[0] * b[1] - a[1] * b[0]

	div = det(xdiff, ydiff)
	if div == 0:
		div = div_backup
	div_backup = div

	d = (det(*line1), det(*line2))
	x = det(d, xdiff) / div
	y = det(d, ydiff) / div

	return [x, y]


def cal_angle(points):
	intersection = points[0]
	mid = points[1]
	
	slope = (float)(intersection[0] - mid[0]) / (mid[1] - intersection[1])

	if slope > 0:
		angle = np.arctan(slope) * 180 / np.pi
		if angle < -50:
			angle = -50
		elif angle > 50:
			angle = 50
		print("angle: {}".format(angle))
		return -angle
	else :
		angle = np.arctan(slope) * 180 / np.pi
		if angle < -50:
			angle = -50
		elif angle > 50:
			angle = 50
		print("angle: {}".format(angle))
		return -angle		
		

def draw_line_and_find_angle(linearray, image):
	global line1_backup, line2_backup, angle_backup

	print("There're {} lines...".format(len(linearray)))
	if len(linearray) == 1:
		angle = cal_angle([[linearray[0][0], linearray[0][1]], [linearray[0][2], linearray[0][3]]])
		if angle < 0:
			x1_1 = line1_backup[0]
			y1_1 = line1_backup[1]
			x1_2 = line1_backup[2]
			y1_2 = line1_backup[3] 

			x2_1 = linearray[-1][0]
			y2_1 = linearray[-1][1]
			x2_2 = linearray[-1][2]
			y2_2 = linearray[-1][3]      
			cv2.line(image, (x2_1, y2_1), (x2_2, y2_2), (255, 0, 0), 3)
			cv2.rectangle(image, (x2_1 - 5, y2_1 - 5), (x2_1 + 5, y2_1 + 5), (0, 255, 0), 2)

			line2_backup = copy.deepcopy(linearray[-1])
		else: 
			x1_1 = linearray[0][0]
			y1_1 = linearray[0][1]
			x1_2 = linearray[0][2]
			y1_2 = linearray[0][3]      
			cv2.line(image, (x1_1, y1_1), (x1_2, y1_2), (255, 0, 0), 3)
			cv2.rectangle(image, (x1_1 - 5, y1_1 - 5), (x1_1 + 5, y1_1 + 5), (0, 255, 0), 2)

			line1_backup = linearray[0]

			x2_1 = line2_backup[0]
			y2_1 = line2_backup[1]
			x2_2 = line2_backup[2]
			y2_2 = line2_backup[3]

		x3_1 = (x1_1 + x2_1) // 2
		y3_1 = (y1_1 + y2_1) // 2
		cv2.rectangle(image, (x3_1 - 5, y3_1 - 5), (x3_1 + 5, y3_1 + 5), (0, 255, 0), 2)

		cv2.rectangle(image, (Width // 2 - 5, y3_1 - 5), (Width // 2 + 5, y3_1 + 5), (0, 0, 255), 2)

		intersection = find_intersection_point([[x1_1, y1_1], [x1_2, y1_2]], [[x2_1, y2_1], [x2_2, y2_2]])
		angle = angle_backup

		return angle	
	else:
		x1_1 = linearray[0][0]
		y1_1 = linearray[0][1]
		x1_2 = linearray[0][2]
		y1_2 = linearray[0][3]      
		cv2.line(image, (x1_1, y1_1), (x1_2, y1_2), (255, 0, 0), 3)
		cv2.rectangle(image, (x1_1 - 5, y1_1 - 5), (x1_1 + 5, y1_1 + 5), (0, 255, 0), 2)

		line1_backup = linearray[0]

		x2_1 = linearray[-1][0]
		y2_1 = linearray[-1][1]
		x2_2 = linearray[-1][2]
		y2_2 = linearray[-1][3]      
		cv2.line(image, (x2_1, y2_1), (x2_2, y2_2), (255, 0, 0), 3)
		cv2.rectangle(image, (x2_1 - 5, y2_1 - 5), (x2_1 + 5, y2_1 + 5), (0, 255, 0), 2)

		line2_backup = copy.deepcopy(linearray[-1])

		intersection = find_intersection_point([[x1_1, y1_1], [x1_2, y1_2]], [[x2_1, y2_1], [x2_2, y2_2]])

		x3_1 = (x1_1 + x2_1) // 2
		y3_1 = (y1_1 + y2_1) // 2
		cv2.rectangle(image, (x3_1 - 5, y3_1 - 5), (x3_1 + 5, y3_1 + 5), (0, 255, 0), 2)

		cv2.rectangle(image, (Width // 2 - 5, y3_1 - 5), (Width // 2 + 5, y3_1 + 5), (0, 0, 255), 2)

		angle = cal_angle([[intersection[0], intersection[1]], [Width // 2, y3_1]])
		angle_backup = angle

		return angle	


def draw_steer(image, steer_angle):
    global Width, Height, arrow_pic

    arrow_pic = cv2.imread('steer_arrow.png', cv2.IMREAD_COLOR)

    origin_Height = arrow_pic.shape[0]
    origin_Width = arrow_pic.shape[1]
    steer_wheel_center = origin_Height * 0.74
    arrow_Height = Height/2
    arrow_Width = (arrow_Height * 462) / 728

    matrix = cv2.getRotationMatrix2D((origin_Width / 2, steer_wheel_center), (steer_angle) * 1.5, 0.7)    
    arrow_pic = cv2.warpAffine(arrow_pic, matrix, (origin_Width + 60, origin_Height))
    arrow_pic = cv2.resize(arrow_pic, dsize = (arrow_Width, arrow_Height), interpolation = cv2.INTER_AREA)

    gray_arrow = cv2.cvtColor(arrow_pic, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray_arrow, 1, 255, cv2.THRESH_BINARY_INV)

    arrow_roi = image[arrow_Height: Height, (Width / 2 - arrow_Width / 2) : (Width / 2 + arrow_Width / 2)]
    arrow_roi = cv2.add(arrow_pic, arrow_roi, mask=mask)
    res = cv2.add(arrow_roi, arrow_pic)
    image[(Height - arrow_Height): Height, (Width / 2 - arrow_Width / 2): (Width / 2 + arrow_Width / 2)] = res


# You are to publish "steer_anlge" following load lanes
if __name__ == '__main__':

	rospy.init_node('line_drive')

	cap = cv2.VideoCapture('kmu_track.mkv')
	time.sleep(3)
	mid_y_roi = (roi[0][0][1] + roi[0][1][1]) // 2

	while cap.isOpened():
		while not rospy.is_shutdown():
			ret, image = cap.read()
			result = image.copy()
			if ret == False:
				break

			blur = cv2.GaussianBlur(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (3, 3), 0)
			canny = cv2.Canny(blur, 100, 180)
			roi_image = set_roi(canny, roi)

			linearr = HoughLines(roi_image, mid_y_roi)

			angle = 0
			if len(linearr) != 0:
				mainline = find_main_line(linearr, 50)
				angle = draw_line_and_find_angle(mainline, result)
			
			steer_angle = angle
			draw_steer(result, steer_angle)
			cv2.imshow('result', result)

			if cv2.waitKey(1) & 0xFF == 27:
				break
		break
	cap.release()
	cv2.destroyAllWindows()

	print("Video is over, Press ctrl-c")
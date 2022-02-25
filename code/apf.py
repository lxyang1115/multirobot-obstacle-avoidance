#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import actionlib
import numpy as np 
from cv_bridge import CvBridge, CvBridgeError
from actionlib_msgs.msg import*
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Pose, Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from tf import transformations
from math import *
import tf

pub_ = None
d = 0.6
sum_error = 0.0

Ranges = None
max_detection_d = 1
goal_d = 0
goal_x = 3
goal_y = 5
kr = 1
kg = 10
f2v = 0.002
t2w = 0.4

regions_ = {
	'right':0,
	'fright':0,
	'front':0,
	'fleft':0,
	'left':0,
} # 激光雷达的五个区域


# 计算障碍物产生的斥力
def rep_f():
	var = rospy.wait_for_message('/turtlebot/laser/scan',LaserScan,timeout=5) # 读取激光雷达数据
	Ranges = var.ranges
	rx = 0
	ry = 0
	for i in range(len(Ranges)):
		# 当距离超过最大探测距离时，选择跳过
		if Ranges[i] > max_detection_d: 
			continue
		# 大小与距离的二次方成反比，并加以一定的权重
		f = kr * (1/Ranges[i])**2 
		rx -= f * sin(1.0*i/len(Ranges)*3.1415926) # 分量
		ry += f * cos(1.0*i/len(Ranges)*3.1415926)
	print "rx: %f ry: %f" % (rx, ry)
	return (rx, ry)

# 获取机器人位置
def get_pos():
	var = rospy.wait_for_message('/odom',Odometry,timeout=5)
	data = var.pose.pose
	# 四元数变换
	(roll, pitch, yaw) = transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
	# 返回x，y坐标及绕z轴转角yaw
	return (data.position.x, data.position.y, yaw)

# 计算目标点产生的引力		
def gra_f():
	global goal_d
	(x, y, th) = get_pos()
	goal_d = sqrt((goal_x-x)*(goal_x-x)+(goal_y-y)*(goal_y-y))
	if goal_d < 0.5:
		kg = 20000
	else:
		kg = 200
	# 计算分量，并加以一定的权重
	gx = ((goal_x - x) * cos(th) + (goal_y - y) * sin(th)) / goal_d * kg
	gy = (-(goal_x - x) * sin(th) + (goal_y - y) * cos(th)) / goal_d * kg
	print "gx: %f gy: %f" % (gx, gy)
	print "x: %f y: %f" % (x, y)
	return (gx, gy)
	
def main():
	global pub_, fx, fy, goal_x, goal_y
	goal_x = float(input("goal x:"))
	goal_y = float(input("goal y:"))
	rospy.init_node('reading_laser',anonymous=True)
	pub_ = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
	#sub = rospy.Subscriber('/turtlebot/laser/scan', LaserScan, get_dis)
	#pos = rospy.Subscriber("/robot_pose", Pose, get_pos)
	rate = rospy.Rate(5)
	while True:
		rospy.sleep(1)
		msg = Twist()
		(rx, ry) = rep_f()
		(gx, gy) = gra_f()
		# 计算合力
		fx = rx + gx
		fy = ry + gy
		#f = sqrt(fx*fx + fy*fy) * abs(fx) / fx
		#msg.linear.x = f*f2v
		angle = atan2(fy,fx)  # left>0 right<0
		# 固定速度大小，转角与合力与x轴夹角成正比
		msg.linear.x = 0.3
		msg.angular.z = angle*t2w
		if goal_d < 1:
			msg.linear.x = 0.1
			msg.angular.z = angle*0.5
		pub_.publish(msg)
		if goal_d < 0.1:
			print "The robot arrives at the goal point: (%f, %f)" % (goal_x, goal_y)
			break
		print "fx: %f fy: %f angle: %f" % (fx, fy, angle)


if __name__ == "__main__":
    main()


#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import actionlib
import numpy as np 
import math

from cv_bridge import CvBridge, CvBridgeError
from actionlib_msgs.msg import*
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Pose, Twist, Point, Quaternion,PoseStamped
from nav_msgs.msg import Odometry
from tf import transformations
from math import *
import tf

pub_ = None
d = 0.6
sum_error = 0.0

Ranges = None
max_detection_d = 2.5#最大探测距离
goal_d = 0#机器人当前距离目标点的距离
goal_x = 3#目标点x坐标
goal_y = 5#目标点y坐标
kr = 1 # 斥力系数
kg = 10 #引力系数
#f2v = 0.002
#t2w = 0.4
cr_d = 6 #引力分段函数的分段点

regions_ = {
	'right':0,
	'fright':0,
	'front':0,
	'fleft':0,
	'left':0,
} # 激光雷达的五个区域


# 计算障碍物产生的斥力
def rep_f(rot):
	var = rospy.wait_for_message('/turtlebot/laser/scan',LaserScan,timeout=5) # 读取激光雷达数据
	Ranges = var.ranges
	rx = 0
	ry = 0
	(roll, pitch, yaw) = transformations.euler_from_quaternion(rot)
	print(roll,pitch,yaw)
	for i in range(len(Ranges)):
		# 当距离超过最大探测距离时，选择跳过
		if Ranges[i] > max_detection_d: 
			continue
		# 大小与距离的二次方成反比，并加以一定的权重
		
		f = kr * (1/Ranges[i])**2 
		rx -= f * sin(1.0*i/len(Ranges)*3.1415926+yaw) # x方向分量斥力
		ry += f * cos(1.0*i/len(Ranges)*3.1415926+yaw) # y方向分量斥力
	rx = threshold(rx,-100,100)#限制斥力大小再-100到100
	ry = threshold(ry,-100,100)
	print "rx: %f ry: %f" % (rx, ry)
	return (rx, ry)
def threshold(x,min_,max_):#阈值函数，用于防止超出范围
	if x<min_:
		return min_
	elif x > max_:
		return max_
	else:
		return x
# 获取机器人位置和速度
def get_pos_and_velocity():
	var = rospy.wait_for_message('/odom',Odometry,timeout=5)#
	data = var.pose.pose#机器人位置
	v = var.twist.twist.linear.x#机器人速度
	w = var.twist.twist.angular.z#机器人角速度
	# 四元数变换
	(roll, pitch, yaw) = transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
	# 返回x，y坐标及绕z轴转角yaw
	return (data.position.x, data.position.y, yaw,v,w)

# 计算目标点产生的引力		
def gra_f():
	global goal_d
	global cr_d
	global kg
	(x, y, th, v, w) = get_pos_and_velocity()
	goal_d = sqrt((goal_x-x)*(goal_x-x)+(goal_y-y)*(goal_y-y))#计算距离目标点距离
	if goal_d < cr_d and goal_d >2:#距离目标点适中，引力与距离成正比
		gx = kg * (goal_x - x)
		gy = kg * (goal_y - y)
	elif goal_d >= cr_d:#距离目标点较远，防止引力过大，取恒定值
		gx = kg * cr_d * (goal_x - x)/sqrt((goal_x -x)**2 + (goal_y - y)**2)
		gy = kg * cr_d * (goal_y - y)/sqrt((goal_x -x)**2 + (goal_y - y)**2)
	else:#距离目标点足够近，增大引力达到目标（防止引力过小）
		gx = 5*kg * cr_d * (goal_x - x)/sqrt((goal_x -x)**2 + (goal_y - y)**2)
		gy = 5*kg * cr_d * (goal_y - y)/sqrt((goal_x -x)**2 + (goal_y - y)**2)
	# 计算分量，并加以一定的权重
	#gx = ((goal_x - x) * cos(th) + (goal_y - y) * sin(th)) / goal_d * kg
	#gy = (-(goal_x - x) * sin(th) + (goal_y - y) * cos(th)) / goal_d * kg
	print "gx: %f gy: %f" % (gx, gy)
	print "x: %f y: %f" % (x, y)
	return (gx, gy)
def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
	# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
	# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)	
def main():
	global pub_, fx, fy, goal_x, goal_y
	goal_x = float(input("goal x:"))#输入目标点x
	goal_y = float(input("goal y:"))#输入目标点y
	rospy.init_node('reading_laser',anonymous=True)
	listener = tf.TransformListener()#监听tf
	pub_ = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
	#sub = rospy.Subscriber('/turtlebot/laser/scan', LaserScan, get_dis)
	#pos = rospy.Subscriber("/robot_pose", Pose, get_pos)
	(x, y, th,v,w) = get_pos_and_velocity()
	delta = atan2(goal_y-y,goal_x-x) - th#计算当前朝向与目标点朝向偏差
	while abs(delta) > 0.1:#超前过大，需要回正，防止倒着走
		msg = Twist()
		msg.linear.x = 0
		msg.angular.z = abs(delta)/delta*0.4
		pub_.publish(msg)
		(x, y, th,v,w) = get_pos_and_velocity()
		delta = atan2(goal_y-y,goal_x-x) - th
	rate = rospy.Rate(10)
	times=0
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		msg = Twist()
		
		(rx, ry) = rep_f(rot)
		(gx, gy) = gra_f()
		# 计算合力
		fx = rx + gx
		fy = ry + gy
		(x, y, th,v,w) = get_pos_and_velocity()
		ft = fx * cos(th) + fy * sin(th)#分解到机器人的base_link 坐标系下
		fn = fy * cos(th) - fx * sin(th)#分解到机器人的base_link 坐标系下
		#f = sqrt(fx*fx + fy*fy) * abs(fx) / fx
		#msg.linear.x = f*f2v
		angle = atan2(fn,ft)  # left>0 right<0
		
		msg.linear.x = threshold(v +  0.002 *ft,-0.12,0.65)#计算下一时刻的速度
		msg.angular.z = threshold(fn / msg.linear.x * 0.003,-0.4,0.4)#计算下一时刻的角速度
		if msg.linear.x<0:
			times+=1
		else :
			times=0
		if times>20:#长时间处于倒车
			(x, y, th,v,w) = get_pos_and_velocity()
			delta = atan2(goal_y-y,goal_x-x) - th
			while abs(delta) > 0.1:#回正
				msg = Twist()
				msg.linear.x = 0
				msg.angular.z = abs(delta)/delta*0.4
				pub_.publish(msg)
				(x, y, th,v,w) = get_pos_and_velocity()
				delta = atan2(goal_y-y,goal_x-x) - th
		
		
		print("v",msg.linear.x,"w",msg.angular.z)
		if goal_d < 1:#距离目标点很近，减慢速度
			msg.linear.x = 0.1
			msg.angular.z = angle*0.5
		pub_.publish(msg)
		if goal_d < 0.2:#考虑到误差，认为到达目标点
			print "The robot arrives at the goal point: (%f, %f)" % (goal_x, goal_y)
			break
		print "fx: %f fy: %f angle: %f" % (fx, fy, angle)
		print("goal_d",goal_d)
		rate.sleep()

if __name__ == "__main__":
    main()


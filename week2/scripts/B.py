#!/usr/bin/env python

import rospy
import random
import math
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from week2_170040012_190100057.msg import Error
from tf.transformations import euler_from_quaternion

current_x = 0.0
current_y = 0.0
current_theta = 0.0

goal_x = 0.0
goal_y = 0.0
t=0.0
desired_theta = 0.0

E_pos = 0.0
E_theta = 0.0
delta_x = 0.0
delta_y = 0.0


error_pub = rospy.Publisher('/error', Error, queue_size=5)

def quat2euler(x,y,z,w):
    quat = [x,y,z,w]
    return euler_from_quaternion(quat)  # in radians

def distance(x1,y1,x2,y2):
	return math.sqrt( (x1-x2)**2 + (y1-y2)**2 )

def error_publisher():
	e=Error()
	e.E_pos=E_pos
	e.E_theta=E_theta
	e.delta_x=desired_theta
	# e.delta_y=delta_y
	e.delta_y=current_theta

	error_pub.publish(e)
	# print('eeeeeeeeeeee')

def odom_callback(data):
	global current_x, current_y, current_theta, pose
	current_x = data.pose.pose.position.x
	current_y = data.pose.pose.position.y
	x = data.pose.pose.orientation.x
	y = data.pose.pose.orientation.y
	z = data.pose.pose.orientation.z
	w = data.pose.pose.orientation.w
	# pose = [data.pose.pose.position.x, data.pose.pose.position.y, quat2euler(x,y,z,w)[2]]
	current_theta=quat2euler(x,y,z,w)[2]  ### in radians
	# print('oooooooooo')

def waypoint_callback(data):
	global goal_x, goal_y, desired_theta, E_theta, E_pos, delta_x, delta_y
	goal_x = data.x
	goal_y = data.y
	t = data.z
	K = 5

	# if (goal_y-current_y)<-0.1 and (goal_x-current_x)<-0.1:
	# 	E_pos = -1*distance(current_x, current_y, goal_x, goal_y)
	# else:
	E_pos = distance(current_x, current_y, goal_x, goal_y)

	delta_x = goal_x-current_x
	delta_y = goal_y-current_y
	epsilon = 0.000001

	desired_theta = math.asin((goal_y-current_y)/(E_pos))
	if (t>0 and t<=9) or (t>27 and t<=36):
		desired_theta = math.pi - desired_theta
	if (t>9 and t<=27):
		desired_theta = -math.pi - desired_theta
	# desired_theta = math.atan((-2*math.cos(2*(t)*K*math.pi/180)) / (math.sin((t)*K*math.pi/180) - epsilon)) ## adding epsilon to prevent division from zero error
	# if goal_x>0 and goal_y>0:
	# 	desired_theta = math.pi + desired_theta
	# if desired_theta < 0:
	# 		desired_theta = math.pi + desired_theta 
	E_theta = desired_theta - current_theta 

	if t>54:
		E_pos=0
		E_theta=0
	
	# if goal_y*current_y < 0 or goal_x*current_x < 0:
	# 	E_theta = -1*E_theta
	
	# if current_theta <= math.pi:
	# 	if desired_theta < 0:
	# 		E_theta = math.pi + desired_theta - current_theta
	# 	else:
	# 		E_theta = desired_theta - current_theta
	# else:
	# 	if desired_theta < 0:
	# 		E_theta = math.pi + desired_theta - (2*math.pi - current_theta)
	# 	else:
	# 		E_theta = desired_theta - (2*math.pi - current_theta)

	
	# E_theta = math.atan((4*math.sin(2*t+4*math.pi/180) - 4*math.sin(2*t)) / (4*math.cos(t+2*math.pi/180) - 4*math.cos(t))) - (current_theta)
	error_publisher()
	
def odom_subscriber():
	rospy.init_node('B')
	odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
	# print('5555')
	waypoint_sub = rospy.Subscriber('/waypoint', Point, waypoint_callback)
	rospy.spin()


if __name__ == '__main__':
	odom_subscriber()
	# print('bbbbbbb')

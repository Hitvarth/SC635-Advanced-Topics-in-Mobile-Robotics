#!/usr/bin/env python

import rospy
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
	e.desired_theta=desired_theta
	e.current_theta=current_theta

	error_pub.publish(e)

def odom_callback(data):
	global current_x, current_y, current_theta, pose
	current_x = data.pose.pose.position.x
	current_y = data.pose.pose.position.y
	x = data.pose.pose.orientation.x
	y = data.pose.pose.orientation.y
	z = data.pose.pose.orientation.z
	w = data.pose.pose.orientation.w
	current_theta=quat2euler(x,y,z,w)[2]  ### in radians
	
E_pos_arr = list()
E_theta_arr = list()
t_arr = list()

def waypoint_callback(data):
	global goal_x, goal_y, desired_theta, E_theta, E_pos, delta_x, delta_y, E_pos_arr, E_theta_arr, t_arr, current_theta
	goal_x = data.x
	goal_y = data.y
	t = data.z
	K = 5

	E_pos = distance(current_x, current_y, goal_x, goal_y)

	desired_theta = math.asin((goal_y-current_y)/(E_pos))

	if (t>0 and t<=9) or (t>27 and t<=36):
		desired_theta = math.pi - desired_theta
	if (t>9 and t<=27):
		desired_theta = -math.pi - desired_theta

	if current_theta>0:
		E_theta = desired_theta - (current_theta - math.pi)
		
	elif current_theta<=0:
		E_theta = desired_theta - (current_theta + math.pi)

	E_pos_arr.append(E_pos)
	E_theta_arr.append(E_theta)
	t_arr.append(t)

	if t>54:
		E_pos=0
		E_theta=0
		plt.xlabel('waypoint index')
		plt.ylabel('error')
		plt.plot(t_arr, E_theta_arr)
		# plt.legend("E theta")
		plt.plot(t_arr, E_pos_arr)
		# plt.legend("E_pos")
		plt.show()	
	
	error_publisher()
	
def odom_subscriber():
	rospy.init_node('B')
	odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
	waypoint_sub = rospy.Subscriber('/waypoint', Point, waypoint_callback)
	rospy.spin()


if __name__ == '__main__':
	odom_subscriber()
	
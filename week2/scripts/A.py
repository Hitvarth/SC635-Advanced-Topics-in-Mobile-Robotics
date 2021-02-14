#!/usr/bin/env python

import rospy
import random
import math
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion


t = -17   # so that it starts from the point next to the origin in the fourth quadrant
K = 5	  # increment in degrees

current_x=0.0
current_y=0.0
threshold = 0.07

waypoint_pub=rospy.Publisher('/waypoint',Point,queue_size=10)

def waypoint_publisher():
	global current_x, current_y, current_theta, t 
	x = 4*math.cos(t*K*math.pi/180)
	y = 4*math.sin(2*t*K*math.pi/180)
	if abs(current_x - x)<=threshold and abs(current_y - y)<=threshold:
		t+=1
		x = 4*math.cos(t*K*math.pi/180)
		y = 4*math.sin(2*t*K*math.pi/180)
	p=Point()
	p.x=4*math.cos(t*K*math.pi/180)
	p.y=4*math.sin(2*t*K*math.pi/180)
	p.z=t
	# while not rospy.is_shutdown:
	waypoint_pub.publish(p)


X=list()
Y=list()

def get_current_pos(data):
	global current_x, current_y, current_theta, X, Y
	current_x = data.pose.pose.position.x
	current_y = data.pose.pose.position.y
	waypoint_publisher()
	X.append(current_x)
	Y.append(current_y)
	# # plot the /odom data 
	if t>54:
		plt.plot(X,Y)
		plt.show()


def generate_waypoints():	
	rospy.init_node('A',anonymous=True)
	odom_sub0 = rospy.Subscriber('/odom', Odometry, get_current_pos)
	rospy.spin()
	

if __name__=='__main__':
	generate_waypoints()



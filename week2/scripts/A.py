#!/usr/bin/env python

import rospy
import random
import math
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion



# i=0

# def generate_waypoints():
# 	X=[4*math.cos(0.001*t) for t in range(0,10000)]
# 	Y=[4*math.sin(2*0.001*t) for t in range(0,10000)]
# 	# plt.plot(X,Y)
# 	# plt.show()
# 	waypoint_pub=rospy.Publisher('/waypoint',Point,queue_size=5)
# 	rospy.init_node('A',anonymous=True)
# 	rate = rospy.Rate(20)
# 	while not rospy.is_shutdown():
# 		# i=random.randint(0,99)
# 		global i
# 		p=Point()
# 		p.x=X[i]
# 		p.y=Y[i]
# 		p.z=0.0
# 		# print('2222222')
# 		waypoint_pub.publish(p)
# 		i+=1
# 		rate.sleep()


# if __name__=='__main__':
# 	try:
# 		generate_waypoints()
# 	except rospy.ROSInterruptException:
# 		pass

t = -18   # so that it starts from the origin # 90 degrees
# t=0.0
K = 5 # increment in degrees

# x = 0

current_x=0.0
current_y=0.0
# current_theta=0.0
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

# def waypoint_publisher():
# 	global current_x, current_y, current_theta, x

# 	for i in range(1,401):

# 		# 4th quadrant
# 		if ()

	 
	
# 	if abs(current_x - x)<=threshold and abs(current_y - y)<=threshold:
# 		t+=1
# 		x = 4*math.cos(t*K*math.pi/180)
# 		y = 4*math.sin(2*t*K*math.pi/180)
# 	p=Point()
# 	p.x=4*math.cos(t*K*math.pi/180)
# 	p.y=4*math.sin(2*t*K*math.pi/180)
# 	p.z=t
# 	# while not rospy.is_shutdown:
# 	waypoint_pub.publish(p)



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
		plt.show(block=True)



def generate_waypoints():	
	rospy.init_node('A',anonymous=True)
	odom_sub0 = rospy.Subscriber('/odom', Odometry, get_current_pos)
	rospy.spin()
	

if __name__=='__main__':
	generate_waypoints()


# how to plot in B?


	# rospy.init_node('A',anonymous=True)
	# odom_sub0 = rospy.Subscriber('/odom', Odometry, queue_size=5)

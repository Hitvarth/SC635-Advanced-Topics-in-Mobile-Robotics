#!/usr/bin/env python

import rospy
import math
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from week2_170040012_190100057.msg import Error

K1=10
K2=5
theta_threshold=5*math.pi/180

control_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

def control_law(data):
	E_pos = data.E_pos
	E_theta = data.E_theta
	
	velocity_command = Twist()
	
	velocity_command.linear.x = -1*min((K1)*E_pos,0.8)
	velocity_command.angular.z = K2*E_theta

	if abs(E_theta)>theta_threshold:
		velocity_command.linear.x = 0
		velocity_command.linear.y = 0
		velocity_command.angular.z = K2*E_theta
	

	control_pub.publish(velocity_command)

def error_subscriber():
	rospy.init_node('C')
	error_sub = rospy.Subscriber('/error', Error, control_law)
	rospy.spin()


if __name__ == '__main__':
	error_subscriber()

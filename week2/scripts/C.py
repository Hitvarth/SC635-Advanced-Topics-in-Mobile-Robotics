#!/usr/bin/env python

import rospy
import random
import math
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from week2_170040012_190100057.msg import Error

K1=0.7
K2=5
# theta_threshold=10*math.pi/180

control_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

def control_law(data):
	E_pos = data.E_pos
	E_theta = data.E_theta
	delta_x = data.delta_x
	delta_y = data.delta_y
	
	velocity_command = Twist()

	# if abs(delta_x)<=0.5:
	# 	epsilon_x=0.5
	# else:
	# 	epsilon_x=0.0
	# if abs(delta_y)<=0.5:
	# 	epsilon_y=0.5
	# else:
	# 	epsilon_y=0.0

	# if abs(E_theta)<=theta_threshold:
	# 	velocity_command.linear.x = K1*delta_x + epsilon_x
	# 	velocity_command.linear.y = K1*delta_y + epsilon_y
	# 	velocity_command.angular.z = 0.0
	# else:
	# 	velocity_command.linear.x = 0.0
	# 	velocity_command.linear.y = 0.0
	# 	velocity_command.angular.z = -1*K2*E_theta

	# if E_pos < 0:
	# 	velocity_command.linear.x = (K1)*E_pos
	# 	velocity_command.angular.z = K2*E_theta

	# else:
	# 	velocity_command.linear.x = min( K1*E_pos, 0.8)
	# 	# velocity_command.linear.y = K1*delta_y + epsilon_y
	# 	if E_theta>0:
	# 		velocity_command.angular.z = min(K2*E_theta, 1)
	# 	if E_theta<=0:
	# 		velocity_command.angular.z = max(K2*E_theta, -1)

	velocity_command.linear.x = min((K1)*E_pos,0.8)
	velocity_command.angular.z = K2*E_theta

	control_pub.publish(velocity_command)

def error_subscriber():
	rospy.init_node('C')
	error_sub = rospy.Subscriber('/error', Error, control_law)
	rospy.spin()


if __name__ == '__main__':
	error_subscriber()


# set max value of v and omega
### F we should only use linear x message not y
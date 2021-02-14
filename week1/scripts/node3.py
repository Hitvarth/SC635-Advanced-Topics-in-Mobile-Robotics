#!/usr/bin/env python

import rospy
from std_msgs.msg import String

name1='abcd'
name2='pqrs'
roll1='1'
roll2='2'

def NameCallback(data):
	global name1,name2
	name1,name2=data.data.split('_')

def RollCallback(data):
	global roll1,roll2
	roll1,roll2=data.data.split('_')
	print("Student "+name1+" has roll : "+roll1+"\nStudent "+name2+" has roll : "+roll2)

def subscriber():
	rospy.init_node('node3',anonymous=True)
	name_sub=rospy.Subscriber('name',String,NameCallback)
	roll_sub=rospy.Subscriber('roll_num',String,RollCallback)
	# rospy.loginfo("Student "+name1+" has roll : "+roll1+"\nStudent "+name2+" has roll : "+roll2)
	# print("Student "+name1+" has roll : "+roll1+"\nStudent "+name2+" has roll : "+roll2)
	rospy.spin()

if __name__ == '__main__':
	subscriber()
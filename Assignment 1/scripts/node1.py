#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def name_publisher():
	name_pub = rospy.Publisher('name',String,queue_size=5)
	rospy.init_node('node1',anonymous=True)
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		name_str = 'Hitvarth Diwanji_Pranav Deo'
		name_pub.publish(name_str)
		rate.sleep()

if __name__=='__main__':
	try:
		name_publisher()
	except rospy.ROSInterruptException:
		pass






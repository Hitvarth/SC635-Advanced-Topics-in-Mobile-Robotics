#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def roll_publisher():
	rospy.init_node('node2',anonymous=True)
	roll_pub=rospy.Publisher('roll_num',String,queue_size=5)
	roll_str="190100057_170040012"
	rate=rospy.Rate(20)
	while not rospy.is_shutdown():
		roll_pub.publish(roll_str)
		rate.sleep()

if __name__=='__main__':
	try :
		roll_publisher()
	except rospy.ROSInterruptException:
		pass


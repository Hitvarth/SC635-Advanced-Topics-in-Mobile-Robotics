#!/usr/bin/env python

from geometry_msgs.msg import Twist
import rospy


def main():
    rospy.init_node('give_vel')
    pub = rospy.Publisher('/odometry_tracker', Twist, queue_size=1)

    rate = rospy.Rate(5)

    vel = Twist()
    vel.linear.x = 0.11
    vel.angular.z = 0.8

    while not rospy.is_shutdown():
        pub.publish(vel)
        rate.sleep()


if __name__ == '__main__':
    main()

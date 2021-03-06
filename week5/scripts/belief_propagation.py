#! /usr/bin/env python

from nav_msgs.msg import Odometry as Odom
import rospy
from math import *


class point(object):
    def __init__(self, x, y):
        super(point, self).__init__()
        self.x = x
        self.y = y


def callback(msg):
    pass


def algo_motion_model(xt, ut, xt_1, theta, del_t):
    mu = 0.5 * ((xt.x - xt_1.x) * cos(theta) + (xt.y - xt_1.y) * sin(theta))\
        / ((xt.y - xt_1.y) * cos(theta) - (xt.x - xt_1.x) * sin(theta))

    x_star = (xt.x + xt_1.x) / 2 + mu * (xt.y - xt_1.y)
    y_star = (xt.y + xt_1.y) / 2 + mu * (xt_1.x - xt.x)
    r_star = sqrt((xt.x - x_star)**2 + (xt.y - y_star)**2)
    del_theta = atan2(xt_1.y - y_star, xt_1.x - x_star) - atan2(xt.y - y_star, xt.x - x_star)
    v_hat = del_theta / del_t * r_star
    w_hat = del_theta / del_t
    # gamma_hat = 


def main():
    rospy.init_node('give_vel')
    pub = rospy.Publisher('/odometry_tracker', Twist, queue_size=1)
    rospy.Subscriber('/odom', Odom, callback)

    rate = rospy.Rate(5)

    vel = Twist()
    vel.linear.x = 0.11
    vel.angular.z = 0.8

    while not rospy.is_shutdown():
        pub.publish(vel)
        rate.sleep()


if __name__ == '__main__':
    main()

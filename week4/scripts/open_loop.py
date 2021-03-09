#!/usr/bin/env python

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry as Odom
import rospy
import matplotlib.pyplot as plt

odom_x = []
odom_y = []


def callback(msg):
    global odom_x, odom_y
    odom_x.append(msg.pose.pose.position.x)
    odom_y.append(msg.pose.pose.position.y)


def main():
    rospy.init_node('open_loop')
    pub = rospy.Publisher('/odometry_tracker', Twist, queue_size=1)
    rospy.Subscriber('/odom', Odom, callback)
    rate = rospy.Rate(5)

    vel1 = Twist()
    vel1.linear.x = 0.1
    vel1.angular.z = 0.1

    vel = Twist()
    vel.linear.x = 0.1
    vel.angular.z = 0.0

    timestep = 0

    # the timestep and the ros rate manages the velocity timeout
    while not rospy.is_shutdown() and timestep < 25:
        timestep += 1
        if timestep < 2:
            pub.publish(vel1)
        else:
            pub.publish(vel)
        rate.sleep()


if __name__ == '__main__':
    main()
    global odom_x, odom_y
    plt.plot(odom_x, odom_y)
    plt.show()
    print('Distance from (0.5,0) and current position is ' + str((math.sqrt((0.5 - odom_x[-1])**2 + odom_y[-1]**2))))
    print('Difference between (0.5,0) and current position is (' + str(0.5 - odom_x[-1]) + ',' + str(-odom_y[-1]) + ')')

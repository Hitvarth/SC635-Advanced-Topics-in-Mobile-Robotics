#!/usr/bin/env python

from geometry_msgs.msg import Twist
from week5.msg import Pose
from nav_msgs.msg import Odometry
import rospy
import time
import matplotlib.pyplot as plt


odom_x = []
odom_y = []

def ManualOdomCallback(msg):
    global odom_x, odom_y
    odom_x.append(msg.x)
    odom_y.append(msg.y)

def OdomCallback(msg):
    global odom_x, odom_y
    odom_x.append(msg.pose.pose.position.x)
    odom_y.append(msg.pose.pose.position.y)

def main():
    rospy.init_node('give_vel_partC')
    pub = rospy.Publisher('/odometry_tracker', Twist, queue_size=10)
    # pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    sub = rospy.Subscriber('/manual_odom', Pose, ManualOdomCallback)
    # sub = rospy.Subscriber('/odom', Odometry, OdomCallback)

    rate = rospy.Rate(20)

    # start_time=rospy.Time.now() 

    vel1 = Twist()
    vel1.linear.x = 0.1
    vel1.angular.z = 0

    vel2 = Twist()
    vel2.linear.x = 0.1
    vel2.angular.z = 0.1

    vel3= Twist()
    vel3.linear.x = 0
    vel3.angular.z = 0

    t1 = 10.0 #seconds
    t2 = 20.0 #seconds

    # first_run=True
    start_time=rospy.Time.now()
    # start_time=time.time()
    # current_time = time.time()
    current_time = rospy.Time.now()

    while not rospy.is_shutdown() and (current_time.to_sec()-start_time.to_sec()) < t1:
        # if first_run==True:
        #     start_time=rospy.Time.now()
        #     first_run=False
        # current_time = time.time()
        current_time = rospy.Time.now()
        print("start_time in seconds: ",start_time.to_sec())
        print("time in seconds:       ",current_time.to_sec())
        if (current_time.to_sec()-start_time.to_sec()) < t1 :
            pub.publish(vel1)
            print("vel1 published")
        elif (current_time.to_sec()-start_time.to_sec()) < t2:
            pub.publish(vel2)
            print("vel2 published")
        # elif (current_time-start_time) > t2:
        #     pub.publish(vel3)
        #     print("vel3 published")

        rate.sleep()


    global odom_x, odom_y
    plt.plot(odom_x, odom_y)
    plt.show()


if __name__ == '__main__':

    # rospy.sleep(10.)
    main()
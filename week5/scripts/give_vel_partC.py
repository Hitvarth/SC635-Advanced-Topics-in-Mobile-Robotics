#!/usr/bin/env python

from geometry_msgs.msg import Twist
import rospy

def main():
    rospy.init_node('give_vel_partC')
    pub = rospy.Publisher('/odometry_tracker', Twist, queue_size=1)

    rate = rospy.Rate(20)

    # start_time=rospy.Time.now() 

    vel1 = Twist()
    vel1.linear.x = 0.1
    vel1.angular.z = 0

    vel2 = Twist()
    vel2.linear.x = 0.11
    vel2.angular.z = 0.8

    vel3= Twist()
    vel2.linear.x = 0
    vel2.angular.z = 0

    t1 = 5.0 #seconds
    t2 = 10.0 #seconds

    first_run=True
    start_time=rospy.Time.now()

    while not rospy.is_shutdown():
        if first_run==True:
            start_time=rospy.Time.now()
            first_run=False
        time = rospy.Time.now()
        print("start_time in seconds: ",start_time.to_sec())
        print("time in seconds:       ",time.to_sec())
        if (time.to_sec()-start_time.to_sec()) < t1 :
            pub.publish(vel1)
        elif (time.to_sec()-start_time.to_sec()) < t2:
            pub.publish(vel2)
        elif (time.to_sec()-start_time.to_sec()) > t2:
            pub.publish(vel3)

        rate.sleep()


if __name__ == '__main__':

    rospy.sleep(10.)
    main()

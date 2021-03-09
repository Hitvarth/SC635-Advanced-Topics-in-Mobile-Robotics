#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry as Odom
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from week5.msg import Pose
import math

belief_space=list()
for i in range(200):
    row=list()
    for j in range(200):
        row.append(0)
    belief_space.append(row)

iteration=0

current_pose=Pose()
control_velocity=Twist()

def quat2euler(x,y,z,w):
    quat = [x,y,z,w]
    return euler_from_quaternion(quat)

class point(object):
    def __init__(self, x, y):
        super(point, self).__init__()
        self.x = x
        self.y = y

def ManualOdomCallback(msg):
    global current_pose
    current_pose.x=msg.x
    current_pose.y=msg.y
    current_pose.theta=msg.theta

def VelocityCallback(msg):
    global control_velocity
    control_velocity.linear.x=msg.linear.x
    control_velocity.angular.z=msg.angular.z
    
    
# using normal distribution to calculate probability
def prob(a,b):
    epsilon=0.00001
    if b==0:
        p=math.exp(-1*a*a/(2*epsilon))/math.sqrt(2*math.pi*epsilon)
    else:
        p=math.exp(-1*a*a/(2*b))/math.sqrt(2*math.pi*b)
    return p

def algo_motion_model(xt, ut, xt_1, theta, del_t):

    # global iteration

    # if iteration==0 and xt_1.y==0 and xt_1.y==0:
    #     iteration=1;
    #     return 1;

    epsilon=0.0001
    if ((xt.y - xt_1.y) * math.cos(theta) - (xt.x - xt_1.x) * math.sin(theta)) == 0:
        mu = 0.5 * ((xt.x - xt_1.x) * math.cos(theta) + (xt.y - xt_1.y) * math.sin(theta)) / epsilon
    else:
        mu = 0.5 * ((xt.x - xt_1.x) * math.cos(theta) + (xt.y - xt_1.y) * math.sin(theta)) / ((xt.y - xt_1.y) * math.cos(theta) - (xt.x - xt_1.x) * math.sin(theta))

    x_star = (xt.x + xt_1.x) / 2 + mu * (xt.y - xt_1.y)
    y_star = (xt.y + xt_1.y) / 2 + mu * (xt_1.x - xt.x)
    r_star = math.sqrt((xt.x - x_star)**2 + (xt.y - y_star)**2)
    del_theta = math.atan2(xt_1.y - y_star, xt_1.x - x_star) - math.atan2(xt.y - y_star, xt.x - x_star)
    v_hat = abs(del_theta) / del_t * r_star
    w_hat = del_theta / del_t
    # gamma_hat = 
    v = ut.linear.x;
    w = ut.angular.z;
    alpha = 0.01
    return  prob(v-v_hat, alpha*abs(v)+alpha*abs(w)) #* prob(v-v_hat, alpha*abs(v)+alpha*abs(w)) prob(w-w_hat, alpha*abs(v)+alpha*abs(w))


def belief_propagation():
    global iteration, current_pose, control_velocity, belief_space

    origin_x=0
    origin_y=100

    if iteration==0:
        belief_space[origin_x+0][origin_y+0]=1
        fig=plt.figure()
        plt.imshow(belief_space,cmap=cm.gray)
        fig.savefig('/home/hitvarth/Desktop/belief space1.png')
        iteration+=1

    elif iteration==1:
        for x in range(0,200):
            for y in range(-100,100):
                xt_1=Pose()
                xt_1.x=x*0.01  # to convert into meters
                xt_1.y=y*0.01  # to convert into meters
                theta=current_pose.theta
                belief_space[x][y+origin_y]=algo_motion_model(current_pose,control_velocity,xt_1,theta,10)
        print(belief_space)
        fig=plt.figure()
        plt.imshow(belief_space,cmap=cm.gray)
        fig.savefig('/home/hitvarth/Desktop/belief space2.png')
        iteration+=1


    elif iteration==2:
        for x in range(0,200):
            for y in range(-100,100):
                xt_1=Pose()
                xt_1.x=x*0.01
                xt_1.y=y*0.01
                theta=current_pose.theta
                belief_space[x][y+origin_y]=algo_motion_model(current_pose,control_velocity,xt_1,theta,10)
        print(belief_space)
        fig=plt.figure()
        plt.imshow(belief_space,cmap=cm.gray)
        fig.savefig('/home/hitvarth/Desktop/belief space3.png')
        iteration+=1


def main():
    rospy.init_node('belief_propagation')
    # pub = rospy.Publisher('/odometry_tracker', Twist, queue_size=1)
    rospy.Subscriber('/manual_odom', Pose, ManualOdomCallback)
    rospy.Subscriber('/odometry_tracker', Twist, VelocityCallback)

    rate=rospy.Rate(5)

    t1 = 10 #seconds
    t2 = 20 #seconds

    global iteration
    first_run=True

    start_time=rospy.Time.now()

    belief_propagation()

    while not rospy.is_shutdown():
        if first_run==True:
            start_time=rospy.Time.now()
            first_run=False
        time = rospy.Time.now()
        if (time.to_sec()-start_time.to_sec()) > t1-0.1 and iteration==1 :
            belief_propagation()
        elif (time.to_sec()-start_time.to_sec()) > t2-0.1 and iteration==2:
            belief_propagation()
            
        rate.sleep()


if __name__ == '__main__':

    rospy.sleep(10.)
    main()

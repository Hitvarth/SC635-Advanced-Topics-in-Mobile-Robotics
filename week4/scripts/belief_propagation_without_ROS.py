#! /usr/bin/env python

# import rospy
# from nav_msgs.msg import Odometry as Odom
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from week5.msg import Pose
import math
import time

belief_space=list()
for i in range(200):
    row=list()
    for j in range(200):
        row.append(0)
    belief_space.append(row)

iteration=0

current_pose=Pose()
initial_pose=Pose()
control_velocity=Twist()

# def quat2euler(x,y,z,w):
#     quat = [x,y,z,w]
#     return euler_from_quaternion(quat)

# class point(object):
#     def __init__(self, x, y):
#         super(point, self).__init__()
#         self.x = x
#         self.y = y

# def ManualOdomCallback(msg):
#     global current_pose
#     current_pose.x=msg.x
#     current_pose.y=msg.y
#     current_pose.theta=msg.theta

# def VelocityCallback(msg):
#     global control_velocity
#     control_velocity.linear.x=msg.linear.x
#     control_velocity.angular.z=msg.angular.z
    
    
# using normal distribution to calculate probability
def prob(a,b):
    epsilon=0.00001
    if b==0:
        p=math.exp(-1*a*a/(2*epsilon))/math.sqrt(2*math.pi*epsilon)
    else:
        p=math.exp(-1*a*a/(2*b))/math.sqrt(2*math.pi*b)
    return p

# here we have assumed that 
# xt_dash is represented by [x' y' theta'] and 
# xt is represented by [x y theta]
# hence we are calculating p( xt_dash | ut,xt )
def algo_motion_model(xt_dash, ut, xt, theta, del_t):

    epsilon=0.0001
    # if ((xt.y - xt_dash.y) * math.cos(theta) - (xt.x - xt_dash.x) * math.sin(theta)) == 0:
    # if ut.angular.z==0:
    #     mu = 0.5 * ((xt.x - xt_dash.x) * math.cos(theta) + (xt.y - xt_dash.y) * math.sin(theta)) / epsilon
    #     x_star = (xt.x + xt_dash.x) / 2 + mu * (xt.y - xt_dash.y)
    #     y_star = (xt.y + xt_dash.y) / 2 + mu * (xt_dash.x - xt.x)
    #     r_star = math.sqrt((xt.x - x_star)**2 + (xt.y - y_star)**2)
    #     del_theta = math.atan2(xt_dash.y - y_star, xt_dash.x - x_star) - math.atan2(xt.y - y_star, xt.x - x_star)
    #     # v_hat = math.sqrt((xt.x - xt_dash.x)**2 + (xt.y - xt_dash.y)**2)/del_t
    #     v_hat = (abs(del_theta) / del_t) * r_star
    #     w_hat = (del_theta) / del_t
    #     # w_hat=0
    #     v = ut.linear.x
    #     w = ut.angular.z
    #     alpha1 = 0.001
    #     alpha2 = 0.001
    #     return prob(v-v_hat, alpha1*abs(v)+alpha2*abs(w)) * prob(w-w_hat, alpha1*abs(v)+alpha2*abs(w)) #prob(v-v_hat, alpha1*abs(v)+alpha2*abs(w))

    # else:
    if ((xt.y - xt_dash.y) * math.cos(theta) - (xt.x - xt_dash.x) * math.sin(theta)) == 0:
        mu = 0.5 * ((xt.x - xt_dash.x) * math.cos(theta) + (xt.y - xt_dash.y) * math.sin(theta)) / epsilon
    else:
        mu = 0.5 * ((xt.x - xt_dash.x) * math.cos(theta) + (xt.y - xt_dash.y) * math.sin(theta)) / ((xt.y - xt_dash.y) * math.cos(theta) - (xt.x - xt_dash.x) * math.sin(theta))
    x_star = (xt.x + xt_dash.x) / 2 + mu * (xt.y - xt_dash.y)
    y_star = (xt.y + xt_dash.y) / 2 + mu * (xt_dash.x - xt.x)
    r_star = math.sqrt((xt.x - x_star)**2 + (xt.y - y_star)**2)
    del_theta = math.atan2(xt_dash.y - y_star, xt_dash.x - x_star) - math.atan2(xt.y - y_star, xt.x - x_star)
    v_hat = (abs(del_theta) / del_t) * r_star
    w_hat = (del_theta) / del_t
    # gamma_hat = -w_hat
    v = ut.linear.x
    w = ut.angular.z
    alpha1 = 0.001
    alpha2 = 0.001
    return prob(v-v_hat, alpha1*abs(v)+alpha2*abs(w)) * prob(w-w_hat, alpha1*abs(v)+alpha2*abs(w)) #prob(v-v_hat, alpha1*abs(v)+alpha2*abs(w))
    # return prob(w-w_hat, alpha1*abs(v)+alpha2*abs(w)) #prob(v-v_hat, alpha1*abs(v)+alpha2*abs(w))


def belief_propagation():
    global iteration, current_pose, control_velocity, belief_space

    origin_x=0
    origin_y=100

    # if iteration==0:
    belief_space[origin_y+0][origin_x+0]=1
    fig=plt.figure()
    plt.imshow(belief_space,cmap=cm.gray)
    fig.savefig('/home/hitvarth/Desktop/belief space1.png')
    iteration+=1

    # at t = t1 the pose of the robot from manual odom is [1 0 0]
    for i in range(200):
        for j in range(200):
            belief_space[i][j]=0

    # current_pose.x=1
    # current_pose.y=0
    # theta=0

    initial_pose.x=0
    initial_pose.y=0
    theta=0

    control_velocity.linear.x=0.1
    control_velocity.angular.z=0    

    for x in range(0,200):
        for y in range(-100,100):
            xt_dash=Pose()
            xt_dash.x=x*0.01  # to convert into meters
            xt_dash.y=y*0.01  # to convert into meters
            belief_space[-y+origin_y-1][x]=algo_motion_model(xt_dash,control_velocity,initial_pose,theta,10)
    fig=plt.figure()
    plt.imshow(belief_space,cmap=cm.gray)
    plt.show()
    # fig.savefig('/home/hitvarth/Desktop/belief space2 with p1 and p2.png')
    # iteration+=1

    # at t = t2 the pose of the robot from manual odom is [1.86 0.50 1.055]
    for i in range(200):
        for j in range(200):
            belief_space[i][j]=0

    # current_pose.x=1.86
    # current_pose.y=0.50
    # theta=1.055

    initial_pose.x=1
    initial_pose.y=0
    theta=0

    control_velocity.linear.x=0.1
    control_velocity.angular.z=0.1

    # elif iteration==2:
    for x in range(0,200):
        for y in range(-100,100):
            xt_dash=Pose()
            xt_dash.x=x*0.01
            xt_dash.y=y*0.01
            belief_space[-y+origin_y-1][x]=algo_motion_model(xt_dash,control_velocity,initial_pose,theta,10)
    # print(belief_space)
    fig=plt.figure()
    plt.imshow(belief_space,cmap=cm.gray)
    plt.show()
    # fig.savefig('/home/hitvarth/Desktop/belief space3 with p1 and p2.png')
    # iteration+=1


def main():
    # rospy.init_node('belief_propagation')
    # # pub = rospy.Publisher('/odometry_tracker', Twist, queue_size=1)
    # rospy.Subscriber('/manual_odom', Pose, ManualOdomCallback)
    # rospy.Subscriber('/odometry_tracker', Twist, VelocityCallback)

    # rate=rospy.Rate(5)

    # t1 = 10 #seconds
    # t2 = 20 #seconds

    # global iteration
    # first_run=True

    # # start_time=rospy.Time.now()
    # start_time=time.time()

    # belief_propagation()

    # while not rospy.is_shutdown():
    #     # if first_run==True:
    #     #     start_time=rospy.Time.now()
    #     #     first_run=False

    #     current_time = time.time()
    #     if (current_time-start_time) > t1-0.1 and iteration==1 :
    #         belief_propagation()
    #     elif (current_time-start_time) > t2-0.1 and iteration==2:
    #         belief_propagation()
            
    #     rate.sleep()

    belief_propagation()


if __name__ == '__main__':

    main()

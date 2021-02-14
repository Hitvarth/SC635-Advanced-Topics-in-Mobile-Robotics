#!/usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, Twist, Quaternion, Pose2D
from math import atan2, sqrt, cos, sin, pi, copysign
from nav_msgs.msg import Odometry as Odom
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import time
import numpy as np


class give_shapes(object):
  def __init__(self, shape, num_waypoints, center):
    super(give_shapes, self).__init__()
    self.shape = shape
    self.num_waypoints = num_waypoints
    self.center = center

  def get_shape(self):
    return self.get_shape_(self.shape)

  def get_shape_(self, shape):
    if self.shape == 'circle':
      return self.give_circle
    else:
      print('=' * 50)
      print('This shape is not defined')
      print('=' * 50)
      return None

  def give_circle(self, i):
    return (round(4 * cos(2 * pi * i / self.num_waypoints), 4) + self.center.x, round(4 * sin(2 * pi * i / self.num_waypoints), 4) + self.center.y)


class follow_waypoints(object):
  def __init__(self):
    super(follow_waypoints, self).__init__()
    self.kp_ang = 1
    self.kp = 1
    self.v_max = 3
    self.w_max = 1

    self.curr_pose = Pose2D()
    self.curr_orient = Quaternion()

    self.center = Pose2D()

    self.j1, self.j2, self.i = 0, 0, 0
    self.plot_sampler = 0

    self.num_waypoints = 48
    self.shape_class = give_shapes('circle', self.num_waypoints, self.center)
    self.shape = self.shape_class.get_shape()
    self.debug = False

    self.distance_thresh = 0.5

    self.angle_to_goal_old = 0
    self.theta_old = 0
    self.theta_code = 0
    self.theta = 0

    self.x_arr = []
    self.x_shape = []
    self.y_arr = []
    self.y_shape = []

    self.run_node()

  def sign(self, x):
    return copysign(1, x)

  def callback(self, msg):
    rot_q = msg.orientation
    self.curr_orient = rot_q
    _, _, self.theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

  def callback_pos(self, msg):
    if self.center.x == 0 and self.center.y == 0:
      self.center.x = msg.x
      self.center.y = msg.y

  def odom_callback(self, msg):
    self.curr_pose.x = msg.pose.pose.position.x
    self.curr_pose.y = msg.pose.pose.position.y

  def eucledian_dist(self, goal):
    return sqrt((goal.x - self.curr_pose.x)**2 + (goal.y - self.curr_pose.y)**2)

  def plot_shape(self):
    plt.plot(self.x_arr, self.y_arr, 'r', label='Path of bot')
    plt.plot(self.x_shape, self.y_shape, 'g', label='Planned trajectory')
    plt.legend()
    plt.show()

  def run_node(self):
    rospy.Subscriber("/odom", Odom, self.odom_callback)
    rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, self.callback)
    rospy.Subscriber("/robot_pose", Pose2D, self.callback_pos)
    pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)

    speed = Twist()

    r = rospy.Rate(4)

    goal = Point()
    goal.x, goal.y = self.shape(self.i)
    self.x_shape.append(goal.x)
    self.y_shape.append(goal.y)

    while not rospy.is_shutdown():
      inc_x = goal.x - self.curr_pose.x
      inc_y = goal.y - self.curr_pose.y

      if self.plot_sampler % 10 == 0:
        self.x_arr.append(self.curr_pose.x)
        self.y_arr.append(self.curr_pose.y)
      angle_to_goal = atan2(inc_y, inc_x)

      if self.eucledian_dist(goal) >= self.distance_thresh:

        theta_code = self.theta + self.j1 * 2 * pi
        angle_to_goal = angle_to_goal + self.j2 * 2 * pi

        if abs(theta_code - self.theta_old) > 5:
          if self.debug:
            rospy.loginfo("theta_old: " + str(self.theta_old))
            rospy.logwarn("j1: " + str(self.j1))
            rospy.loginfo("theta: " + str(self.theta_code))
          self.j1 = self.j1 + 1
          theta_code = theta_code + 2 * pi
          self.theta_old = theta_code + 2 * self.j1 * pi

        if abs(angle_to_goal - self.angle_to_goal_old) > 5:
          if self.debug:
            rospy.loginfo("angle_old: " + str(self.angle_to_goal_old))
            rospy.loginfo("angle: " + str(angle_to_goal))
            rospy.logwarn("j2: " + str(self.j2))
          self.j2 = self.j2 + 1
          angle_to_goal = angle_to_goal + 2 * pi
          self.angle_to_goal_old = angle_to_goal + self.j2 * 2 * pi

        speed.angular.z = min(self.w_max, max(-self.w_max, self.kp_ang * (angle_to_goal - theta_code)))

        if abs((angle_to_goal - theta_code)) <= 0.1:
          speed.linear.x = min(self.v_max, max(-self.v_max, self.kp * abs(self.eucledian_dist(goal))))
        else:
          speed.linear.x = min(self.v_max, max(-self.v_max, 0.1 * self.kp * abs(self.eucledian_dist(goal))))

        _, _, z, w = quaternion_from_euler(0, 0, theta_code)

        if self.debug:
          rospy.loginfo("angle_goal" + str(angle_to_goal))
          rospy.loginfo("theta     " + str(theta_code))
          rospy.loginfo("theta_old:" + str(self.theta_old))
          rospy.loginfo("x,y:      " + str(self.curr_pose.x) + " " + str(self.curr_pose.y))
          rospy.loginfo("goal x,y: " + str(goal.x) + " " + str(goal.y))
          rospy.loginfo("-" * 30)

        self.angle_to_goal_old = angle_to_goal
        self.theta_old = theta_code

      else:
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        self.i = self.i + 1
        goal.x, goal.y = self.shape(self.i)
        self.x_shape.append(goal.x)
        self.y_shape.append(goal.y)

      pub.publish(speed)
      r.sleep()

      if self.i == self.num_waypoints:
        rospy.signal_shutdown("Circle completed!")


if __name__ == '__main__':

  rospy.init_node("follow_waypoints", disable_signals=True)
  time_s = time.time()
  temp = follow_waypoints()
  rospy.loginfo("Time taken for one revolution is: " + str(time.time() - time_s) + " seconds")
  x_val = np.array(temp.x_arr)
  y_val = np.array(temp.y_arr)
  x_val = x_val - temp.center.x
  y_val = y_val - temp.center.y

  rms = np.sqrt(np.mean(x_val**2 + y_val**2))

  rospy.loginfo("RMS value is: " + str(rms) + " metres")
  temp.plot_shape()
  # rospy.on_shutdown(temp.plot_shape())

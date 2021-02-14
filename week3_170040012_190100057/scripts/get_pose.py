#!/usr/bin/env python
import rospy
from week3.msg import Trilateration
from geometry_msgs.msg import Pose2D
from trilateration import trilaterate, point, circle


def tri_callback(data):
    point1 = point(data.landmarkA.x, data.landmarkA.y)
    circle1 = circle(point1, data.landmarkA.distance + (data.landmarkA.variance)**0.5)

    point2 = point(data.landmarkB.x, data.landmarkB.y)
    circle2 = circle(point2, data.landmarkB.distance + (data.landmarkB.variance)**0.5)

    point3 = point(data.landmarkC.x, data.landmarkC.y)
    circle3 = circle(point3, data.landmarkC.distance + (data.landmarkC.variance)**0.5)

    circle_list = [circle1, circle2, circle3]

    tri = trilaterate()

    inner_points = []
    for p in tri.get_all_intersecting_points(circle_list):
        if tri.is_contained_in_circles(p, circle_list):
            inner_points.append(p)

    center = tri.get_polygon_center(inner_points)
    if center is not False:
        rob_pose = Pose2D()
        rob_pose.x = center.x
        rob_pose.y = center.y
        rob_pose.theta = 0
        pose_pub.publish(rob_pose)


if __name__ == '__main__':
    rospy.init_node('get_pose', anonymous=True)

    # Subscribers
    rospy.Subscriber("trilateration_data", Trilateration, tri_callback)

    # Publishers
    pose_pub = rospy.Publisher('robot_pose', Pose2D, queue_size=10)

    rospy.spin()

#!/usr/bin/env
import math
import rospy


class base_station(object):
  def __init__(self, lat, lon, dist):
    self.lat = lat
    self.lon = lon
    self.dist = dist


class point(object):
  def __init__(self, x, y):
    self.x = x
    self.y = y


class circle(object):
  def __init__(self, point, radius):
    self.center = point
    self.radius = radius


class trilaterate(object):
  def __init__(self):
    super(trilaterate, self).__init__()

  def eucledian_distance(self, p1, p2):
    return math.sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2))

  def get_two_circles_intersecting_points(self, c1, c2):
    p1 = c1.center
    p2 = c2.center
    r1 = c1.radius
    r2 = c2.radius

    d = self.eucledian_distance(p1, p2)
    if d >= (r1 + r2) or d <= math.fabs(r1 - r2):
      return None

    a = (pow(r1, 2) - pow(r2, 2) + pow(d, 2)) / (2 * d)
    h = math.sqrt(pow(r1, 2) - pow(a, 2))
    x0 = p1.x + a * (p2.x - p1.x) / d
    y0 = p1.y + a * (p2.y - p1.y) / d
    rx = -(p2.y - p1.y) * (h / d)
    ry = -(p2.x - p1.x) * (h / d)

    return [point(x0 + rx, y0 - ry), point(x0 - rx, y0 + ry)]

  def get_all_intersecting_points(self, circles):
    points = []
    num = len(circles)
    for i in range(num):
      j = i + 1
      for k in range(j, num):
        res = self.get_two_circles_intersecting_points(circles[i], circles[k])
        if res:
          points.extend(res)
    return points

  def is_contained_in_circles(self, point, circles):
    for i in range(len(circles)):
      if (self.eucledian_distance(point, circles[i].center) > (circles[i].radius)):
        return False
    return True

  def get_polygon_center(self, points):
    center = point(0, 0)
    num = len(points)
    if num == 0:
      rospy.logwarn("No solution for Trilateration")
      return False
    else:
      for i in range(num):
        center.x += points[i].x
        center.y += points[i].y
      center.x /= num
      center.y /= num
      return center

#!/usr/bin/python
## Visual Servo control for AR.Drone

import roslib; roslib.load_manifest('visual_servo')
import rospy;
import re;
from numpy import *

from std_msgs.msg import Empty
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point

FOCAL_LENGTH = 560;
WIDTH = 640;
HEIGHT = 360;
ERRORS = [];
DESIRED = [];
CMD = rospy.Publisher('cmd_vel', Twist)

# Jacobian at image location u,v with depth estimate Z
def getJacobian(projections, nrPoints, Z):
  global WIDTH, HEIGHT, FOCAL_LENGTH
  A = [];
  for k in range(nrPoints):
    p = projections.poses[k].position
    x = (p.x - WIDTH/2)/FOCAL_LENGTH
    y = (p.y - HEIGHT/2)/FOCAL_LENGTH

    # Create two new rows per feature
    row1 = []
    row2 = []
    # omega_x
    row1.append(x*y)
    row2.append(1+y*y)
    # omega_y
    row1.append(-(1+x*y))
    row2.append(-x*y)
    # omega_z
    row1.append(y)
    row2.append(-x)
    # vx
    row1.append(-1/Z)
    row2.append(0)
    # vy
    row1.append(0)
    row2.append(-1/Z)
    # vz
    row1.append(x/Z)
    row2.append(y/Z)
    A.append(row1)
    A.append(row2)

  return A

# Given projection errors, calcuate desired twist
# Kp is the proportional gain
def servo(projections):
  global FOCAL_LENGTH, ERRORS, CMD

  Z = 5
  Kp = 0.001
  nrPoints = 3;

  if len(projections.poses) >= nrPoints:
    calcErrors(projections)

    m = 2*nrPoints;
    A = getJacobian(projections,nrPoints, Z)

    b = []
    for i in range(nrPoints):
      e = ERRORS[len(ERRORS)-1][i]
      b.append(Kp*e.x/FOCAL_LENGTH)
      b.append(Kp*e.y/FOCAL_LENGTH)

    # Solve Ax=b via QR factorization
    Q,R = linalg.qr(A)
    yQR = dot(Q.T, b)
    xQR = linalg.solve(R,yQR)
    # ROS convention for twist(linear, angular) is different from GTSAM (angular,linear)
    twist = Twist()
    twist.angular.x = xQR[0]
    twist.angular.y = xQR[1]
    twist.angular.z = xQR[2]
    twist.linear.x = xQR[3]
    twist.linear.y = xQR[4]
    twist.linear.z = xQR[5]
    CMD.publish(twist)
  else:
    # If we don't have enough features go into autohover mode
    twist = Twist()
    CMD.publish(twist)


# Read in the measured projected features
def calcErrors(projections):
  global ERRORS, DESIRED
  error = []
  for i in range(len(projections.poses)):
    p = projections.poses[i].position
    d = DESIRED[i]
    e = Point(d.x-p.x, d.y-p.y, d.z-p.z)
    error.append(e)
  ERRORS.append(error)


## Read in feature points and publish twist commands to the Drone
def control():
  global DESIRED
  rospy.init_node('vservo_control')
    # Array of poses, but we really only 
  # use the point component as a cheap hack to send arrays of Points
  rospy.Subscriber("features", PoseArray, servo)

  time = rospy.get_time()
  r = rospy.Rate(10.0)

  # TODO: Actual desired config

  d1 = Point(339,79,0)
  d2 = Point(298,174,0)
  d3 = Point(369,174,0)

  DESIRED.append(d1)
  DESIRED.append(d2)
  DESIRED.append(d3)
  rospy.spin()

if __name__ == '__main__':
  try:
    control()
    #test_servo()
  except rospy.ROSInterruptException:
    pass

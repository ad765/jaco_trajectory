#! /usr/bin/env python
""" Built-functions for jaco_control.py to use. Includes mathematical functions, such as matrix multiplication with D-H parameters, inverse kinematics with pseudo-inverse jacobian, and symbolic mathematics. """

import numpy
from scipy.linalg import pinv
from math import pi
from math import cos
from math import sin
from sympy import *
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

def systemKinematics(state):
  # Symbolize state variables
  q1, q2, q3, q4, q5, q6 = symbols('q1 q2 q3 q4 q5 q6')

  # Define geometric parameters for j2n6s300
  D1 = 0.2755
  D2 = 0.4100
  D3 = 0.2073
  D4 = 0.0741
  D5 = 0.0741
  D6 = 0.1600
  e2 = 0.0098

  aa = 30.0*pi/180.0
  ca = cos(aa)
  sa = sin(aa)
  c2a = cos(2*aa)
  s2a = sin(2*aa)
  d4b = D3 + sa/s2a*D4
  d5b = sa/s2a*D4 + sa/s2a*D5
  d6b = sa/s2a*D5 + D6

  # Define DH parameters for all joints
  alpha = numpy.array([pi/2, pi, pi/2, 2*aa, 2*aa, pi])
  a = numpy.array([0, D2, 0, 0, 0, 0])
  d = numpy.array([D1, 0, -e2, -d4b, -d5b, -d6b])
  
  Q1 = DHMatrix(d[0],q1,a[0],alpha[0])
  Q2 = DHMatrix(d[1],q2,a[1],alpha[1])
  Q3 = DHMatrix(d[2],q3,a[2],alpha[2])
  Q4 = DHMatrix(d[3],q4,a[3],alpha[3])
  Q5 = DHMatrix(d[4],q5,a[4],alpha[4])
  Q6 = DHMatrix(d[5],q6,a[5],alpha[5])

  T = Q1 * Q2 * Q3 * Q4 * Q5 * Q6
  base_state = Matrix([[0],[0],[0],[1]])
  dyn = T * base_state
  init_x = dyn.subs(q1,state[0])
  init_x = init_x.subs(q2,state[1])
  init_x = init_x.subs(q3,state[2])
  init_x = init_x.subs(q4,state[3])
  init_x = init_x.subs(q5,state[4])
  init_x = init_x.subs(q6,state[5])
  return dyn, init_x.evalf()

def DHMatrix(d,theta,a,alpha):
  d_mat = Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,d],[0,0,0,1]])
  theta_mat = Matrix([[cos(theta),-sin(theta),0,0],[sin(theta),cos(theta),0,0],[0,0,1,0],[0,0,0,1]])
  a_mat = Matrix([[1,0,0,a],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
  alpha_mat = Matrix([[1,0,0,0],[0,cos(alpha),-sin(alpha),0],[0,sin(alpha),cos(alpha),0],[0,0,0,1]])
  DH_mat = d_mat * theta_mat * a_mat * alpha_mat
  return DH_mat

def robotJacobian(DH_mat):
  # Symbolize state variables
  q1, q2, q3, q4, q5, q6 = symbols('q1 q2 q3 q4 q5 q6')
  q = Matrix([[q1],[q2],[q3],[q4],[q5],[q6]])
  DH_mat_lin = DH_mat.jacobian(q)
  return DH_mat_lin

def evalJacobian(J,state):
  q1, q2, q3, q4, q5, q6 = symbols('q1 q2 q3 q4 q5 q6')
  J = J.subs(q1,state[0])
  J = J.subs(q2,state[1])
  J = J.subs(q3,state[2])
  J = J.subs(q4,state[3])
  J = J.subs(q5,state[4])
  J = J.subs(q6,state[5])
  return J.evalf()

def Sym2NumArray(F):
  #Function to convert symbolic expression with numerical data to numpy array
  shapeF=numpy.shape(F)
  B=zeros(shapeF[0],shapeF[1])
  for i in range(0,shapeF[0]):
    for j in range(0,shapeF[1]):
      B[i,j]=N(F[i,j])
  return B

def inverseKinematics(J,step,delta_X):
  J = Sym2NumArray(J)				# Convert to numpy.array
  J = numpy.array(J).astype(numpy.float64)	# Convert to numpy.matrix
  Jplus = J.transpose()				# Calculate transpose
  Jplus = pinv(J)				# Calculate pseudoinverse
  delta_q = Jplus * delta_X			# Calculate change in joint space
  return delta_q

def moveJoint (jointcmds):
  topic_name = '/j2n6s300/effort_joint_trajectory_controller/command'
  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
  jointCmd = JointTrajectory()  
  point = JointTrajectoryPoint()
  jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
  point.time_from_start = rospy.Duration.from_sec(5.0)
  for i in range(0, 6):
    jointCmd.joint_names.append('j2n6s300_joint_'+str(i+1))
    point.positions.append(jointcmds[i])
    point.velocities.append(0)
    point.accelerations.append(0)
    point.effort.append(0) 
  jointCmd.points.append(point)
  rate = rospy.Rate(1000)
  count = 0
  while (count < 500):
    pub.publish(jointCmd)
    count = count + 1
    rate.sleep()     

def moveFingers(jointcmds):
  topic_name = '/j2n6s300/effort_finger_trajectory_controller/command'
  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)  
  jointCmd = JointTrajectory()  
  point = JointTrajectoryPoint()
  jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
  point.time_from_start = rospy.Duration.from_sec(5.0)
  for i in range(0, 3):
    jointCmd.joint_names.append('j2n6s300_joint_finger_'+str(i+1))
    point.positions.append(jointcmds[i])
    point.velocities.append(0)
    point.accelerations.append(0)
    point.effort.append(0) 
  jointCmd.points.append(point)
  rate = rospy.Rate(1000)
  count = 0
  while (count < 500):
    pub.publish(jointCmd)
    count = count + 1
    rate.sleep()


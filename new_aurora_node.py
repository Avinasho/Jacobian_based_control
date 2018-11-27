'''
Transformation of Tracker information from Aurora
Final version completed 03-04-18
Written by Avinash  Soor
Git: Avinasho
Written for the MEng Individual Project
'''
from __future__ import division, print_function
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point32
from numpy.linalg import inv
from numpy import linalg
import message_filters
import numpy as np
import rospy
import math

################################################################################ MISC.

print(' ')
print('New Aurora Node initialised')
rospy.init_node('New_Final_Aurora')
# creating the node to publish the new transformed position to

################################################################################ CLASS

class AURORA:

    def __init__(self):
        self.pubAur0 = rospy.Publisher('/aurora0', Point32, queue_size=100)
        self.pubAur2 = rospy.Publisher('/final_aurora', Point32, queue_size=100)
        self.pubEul = rospy.Publisher('/Euler_Angles', Point32, queue_size=100)
        # creating the publishers, which send the position to the topic
        self.transformed_aurPos = Point32()
        self.transformed_aurPos_2 = Point32()
        # message types
        self.eulPos = Point32()
        # epsilon for testing whether a number is close to zero
        self._EPS = np.finfo(float).eps * 4.0
        self.T_tip_tracker = [
            [0, 0, 1, -35],
            [0, 1, 0, -7],
            [-1, 0, 0, -47],
            [0, 0, 0, 1] ]
            # the transformation matrix from the tracker to the TCP
        self.iter = 1
        self.Tr0_offset = np.zeros([4, 4])
        self.prevR1 = []
        self.noR1 = 0

################################################################################ FUNCTIONS

    def Tr0_Matrix(self, aurPos, quatPos):
        d0 = aurPos
        Q0 = quatPos
        R0 = self.quaternion_matrix([Q0.x, Q0.y, Q0.z, Q0.w])
        self.prevR1 = R0
        Tr0 = np.zeros([4, 4])
        Tr0[0:3, 0:3] = R0
        Tr0[0, 3] = d0.x
        Tr0[1, 3] = d0.y
        Tr0[2, 3] = d0.z
        Tr0_offset = self.invT( np.matmul(Tr0, self.T_tip_tracker) )
        return Tr0_offset
        # the initial transformation matrix is calculated here

    def quaternion_matrix(self, quaternion):
        q = np.array(quaternion, dtype=np.float64, copy=True)
        n = np.dot(q, q)
        self.noR1 = 0
        if n < self._EPS:
            self.noR1 = 1
            return np.identity(3)
        q *= math.sqrt(2.0 / n)
        q = np.outer(q, q)
        return np.array([
            [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0]],
            [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0]],
            [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2]]])
            # the quaternian positino values are transformed into a rotation
            # matrix here
######################################################################################################
    def mat2euler(self, M, cy_thresh=None):
        '''Parameters
        M : array-like, shape (3,3)
        cy_thresh : None or scalar, optional threshold below which to give up on
                    straightforward arctan for estimating x rotation. If None
                    (default), estimate from precision of input.
        Returns   : Rotations in radians around z, y, x axes, respectively'''
        M = np.asarray(M)
        if cy_thresh is None:
            try:
                cy_thresh = np.finfo(M.dtype).eps * 4
            except ValueError:
                cy_thresh = _FLOAT_EPS_4
        r11, r12, r13, r21, r22, r23, r31, r32, r33 = M.flat
        # cy: sqrt((cos(y)*cos(z))**2 + (cos(x)*cos(y))**2)
        cy = math.sqrt(r33*r33 + r23*r23)
        if cy > cy_thresh: # cos(y) not close to zero, standard form
            z = math.atan2(-r12,  r11) # atan2(cos(y)*sin(z), cos(y)*cos(z))
            y = math.atan2(r13,  cy) # atan2(sin(y), cy)
            x = math.atan2(-r23, r33) # atan2(cos(y)*sin(x), cos(x)*cos(y))
        else: # cos(y) (close to) zero, so x -> 0.0 (see above)
            # so r21 -> sin(z), r22 -> cos(z) and
            z = math.atan2(r21,  r22)
            y = math.atan2(r13,  cy) # atan2(sin(y), cy)
            x = 0.0
        return z, y, x
        # ignore this - was not really utilised
######################################################################################################
    def invT(self, T):
        R = T[0:3, 0:3]
        d = T[0:3, 3]
        T_inv = np.zeros([4,4])
        T_inv[0:3, 0:3] = inv( R )
        T_inv[0:3, 3] = np.matmul( -inv( R ), d )
        return T_inv
        # finds the inverse of the transformation matrix

    def main(self):
        rospy.Subscriber('/aurora', PoseArray, self.aurCallback)
        rospy.spin()
        # subscribes to the position tracker, and begins the transformation loop
        # in the callback function

################################################################################ CALLBACK

    def aurCallback(self, aurData):
        if self.iter == 1:
            self.iter += 1
            pos_curr = aurData.poses[0].position
            quat_curr = aurData.poses[0].orientation
            self.Tr0_offset = self.Tr0_Matrix(pos_curr, quat_curr)
            self.transformed_aurPos.x = self.Tr0_offset[0, 3]
            self.transformed_aurPos.y = self.Tr0_offset[1, 3]
            self.transformed_aurPos.z = self.Tr0_offset[2, 3]
            self.pubAur0.publish(self.transformed_aurPos)
            # at the first iteration the first transformation matrix is found -
            # this is use in all subsequent calculations. this is why this file
            # should be restarted between experiments - to recalibrate the
            # tracking information
        elif self.iter > 1:
            pos_curr_2 = aurData.poses[0].position
            quat_curr_2 = aurData.poses[0].orientation
            R1 = self.quaternion_matrix([quat_curr_2.x, quat_curr_2.y, quat_curr_2.z, quat_curr_2.w])
            # get the rotation matrix from the quaternian values
            if self.noR1 == 0:
                R1 = self.prevR1
            else:
            self.prevR1 = R1
            T1 = np.zeros([4, 4])
            # if np.shape(R1) == (4, 4):
            #     print(R1)
            #     raw_input()
            T1[0:3, 0:3] = R1
            T1[0, 3] = pos_curr_2.x
            T1[1, 3] = pos_curr_2.y
            T1[2, 3] = pos_curr_2.z
            T1[3, 3] = 1
            tot = np.matmul( np.matmul( self.Tr0_offset, T1 ), self.T_tip_tracker )
            self.transformed_aurPos_2.x = tot[0, 3]
            self.transformed_aurPos_2.y = tot[1, 3]
            self.transformed_aurPos_2.z = tot[2, 3]
            self.pubAur2.publish(self.transformed_aurPos_2)
            eul = self.mat2euler(R1)
            self.eulPos.x = eul[1]
            self.eulPos.y = eul[2]
            self.eulPos.z = eul[0]
            self.pubEul.publish(self.eulPos)
            # ignore the euler transformation - it wasn't working
            # this part of the loop is the body of this file - using the initial
            # transformation matrix, all subsequernt positions are transformed
            # in relation to this.

################################################################################ MAIN

print('...')
print('Subscribing to /aurora ...')
print('...')
print('Publishing transformed aurora data to /final_aurora ...')
print('...')
print('Publishing euler angles to /Euler_Angles ...')
print('...')
AURORA().main()

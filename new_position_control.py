'''
P (Proportional) Controller
Final version completed 20-04-18
Written by Avinash  Soor
Git: Avinasho
Written for the MEng Individual Project
'''
from __future__ import division, print_function
from geometry_msgs.msg import Point32
from numpy.linalg import inv
from numpy import linalg
import numpy as np
import rospy
import math
################################################################################ MISC.
print(' ')
print('Position Control Initialised.')
rospy.init_node('Position_Control')
################################################################################ CLASS
class POSITION:
    def __init__(self):
        self.err = Point32()
        self.vel = Point32()
        self.exit_vel = Point32()
        self.pubError = rospy.Publisher('/Error', Point32, queue_size=100)
        self.pubVel = rospy.Publisher('/cmd_vel', Point32, queue_size=100)
        self.goal_position = [4, -1, 5]
        self.goal_position = [2, -1, 8.8]
        self.kp = 0.00001
        self.H_pos = [
            [0.2588, -0.1294, -0.1294],
            [0,       0.2241, -0.2241],
                [0.9659,  0.9659,  0.9659]]
        self.K_pos = [
            [1.0262 * 1e3,             0,            0],
            [           0,  1.0262 * 1e3,            0],
            [           0,             0, 6.3457 * 1e3]]
################################################################################ FUNCTIONS
    def moveRobotTo(self, x, y, z, H, K):
        vec = [-x, y, z]
        V = inv( np.matmul( inv( K ), H ) )
        V = np.matmul(V, vec)
        return V
################################################################################ CALLBACK
    def posCallback(self, aurData):
        aurX = aurData.x
        aurY = aurData.y
        aurZ = aurData.z
        self.err.x = (self.goal_position[0] - aurX)
        self.err.y = (self.goal_position[1] - aurY)
        self.err.z = (self.goal_position[2] - aurZ)
        self.pubError.publish(self.err)
        all_vels = self.moveRobotTo(self.err.x*self.kp, self.err.y*self.kp, self.err.z*self.kp, self.H_pos, self.K_pos)
        self.vel.x = -all_vels[0]
        self.vel.y = -all_vels[1]
        self.vel.z = -all_vels[2]
        self.pubVel.publish(self.vel)
################################################################################ MAIN
    def main(self):
        rospy.Subscriber('/final_aurora',Point32, self.posCallback)
        rospy.spin()
################################################################################
print('...')
print('Subscribing to /final_aurora ...')
print('...')
print('Publishing errors to /Error ...')
print('...')
print('Publishing velocities to /cmd_vel')
print('...')
POSITION().main()

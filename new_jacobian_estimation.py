'''
Jacobian (Estimated) Control
Final version completed 18-05-18
Written by Avinash  Soor
Git: Avinasho
Written for the MEng Individual Project
'''
from __future__ import division, print_function
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from numpy.linalg import inv
from numpy import linalg
import numpy as np
import rospy
import math
# import all the relevent modules
################################################################################ MISC.
print(' ')
print('Jacobian Estimation Initialised')
rospy.init_node('Jacobian_Estimation')
# create the jacobian estimation node
################################################################################ CLASS
class Jacobian_estimation:
    def __init__(self):
        self.what_J = 2 # 1=estimation(identity), 2=analytical, 3=analytical only
        self.pubError = rospy.Publisher('/Error', Point32, queue_size=100)
        self.pubVel = rospy.Publisher('/cmd_vel', Point32, queue_size=100)
        self.pubJac = rospy.Publisher('/Jacobian_Matrix', Float32MultiArray, queue_size=100)
        self.err = Point32()
        self.vel = Point32()
        self.jac2pub = Float32MultiArray()
        self.jac2pub.layout.dim.append(MultiArrayDimension())
        self.jac2pub.layout.dim.append(MultiArrayDimension())
        self.jac2pub.layout.dim[0].label = "height"
        self.jac2pub.layout.dim[1].label = "width"
        self.jac2pub.layout.dim[0].size = 3
        self.jac2pub.layout.dim[1].size = 3
        self.jac2pub.layout.dim[0].stride = 3*3
        self.jac2pub.layout.dim[1].stride = 3
        self.jac2pub.layout.data_offset = 0
        self.error = []
        # self.goal_position = [0, 2, 0]
        self.goal_position = [2, -1, 8.3]
        # self.goal_position = [2, 0, 0]
        # self.goal_position = [3, 0, 0]
        # self.goal_position = [0, 2, 0]
        # self.goal_position = [0, 0, 5]
        # self.goal_position = [-3, 2, 0]
        # self.goal_position = [-3, 0, 5]
        # self.goal_position = [0, -2, 5]
        # self.goal_position = [4, -1, 5]
        # all the goal positions which were tested
        self.curr_x = []
        self.curr_q = []
        self.prev_x = []
        self.prev_q = []
        self.prev_J = np.eye(3)
        self.iter = 1
        self.J_eye = [
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]]
        self.J_analytical = [
            [ 2.6433*1e3,           0, 2.1898*1e3],
            [-1.3217*1e3,  2.2892*1e3, 2.1898*1e3],
            [-1.3217*1e3, -2.2892*1e3, 2.1898*1e3]]
        self.J_analytical = [
            [1, 0 ,0],
            [0, 1, 0],
            [0, 0, 1]]
            # parameter initialisation - the top self.J_analytical is for the
            # pure analytical solution. the bottom one is for the jacobian-based
            # solution (the estimation)
        self.jac2pub.data = self.prev_J
        # this is all just parameter initialisation - I can go into more detail
        # if need be?
################################################################################ FUNCTIONS
    def diff(self, a, b):
        return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]
        #calculates the difference between vectors

    def vec_mult_mat(self, a, b):
        # given a=row, b=col
        return [
            [b[0]*a[0], b[0]*a[1], b[0]*a[2]],
            [b[1]*a[0], b[1]*a[1], b[1]*a[2]],
            [b[2]*a[0], b[2]*a[1], b[2]*a[2]]]
            #calculates the multiplication of two vectores (dot multiplication)

    def vec_mult_val(self, a, b):
        return (a[0]*b[0] + a[1]*b[1] + a[2]*b[2])
        # calculates the multiplication of two vectors

    def mat_div_val(self, vec, val):
        if val == 0:
            val = 1
        return [
            [vec[0][0]/val, vec[0][1]/val, vec[0][2]/val],
            [vec[1][0]/val, vec[1][1]/val, vec[1][2]/val],
            [vec[2][0]/val, vec[2][1]/val, vec[2][2]/val]]
            # divides a matrix by a value

    def mat_mult_val(self, vec, val):
        return [
            [vec[0][0]*val, vec[0][1]*val, vec[0][2]*val],
            [vec[1][0]*val, vec[1][1]*val, vec[1][2]*val],
            [vec[2][0]*val, vec[2][1]*val, vec[2][2]*val]]
            # multiplies a matrix with a vector

    def catch_failed_callback(self, vec):
        if vec == [] or vec == [0.02, 0.02, 0.02]:
            return [0.01, 0.01, 0.01]
        elif vec == [0.01, 0.01, 0.01]:
            return [0.02, 0.02, 0.02]
        else:
            return vec
            # if the callback fails at any point, catch this and return some Value
            # to prevent singularities

    def mat_mult_vec(self, mat, vec):
        row1 = mat[0][0]*vec[0] + mat[0][1]*vec[1] + mat[0][2]*vec[2]
        row2 = mat[1][0]*vec[0] + mat[1][1]*vec[1] + mat[1][2]*vec[2]
        row3 = mat[2][0]*vec[0] + mat[2][1]*vec[1] + mat[2][2]*vec[2]
        return [row1, row2, row3]
        # multiplies a matrix by a vector

    def mat_add(self, m1, m2):
        return [
            [m1[0][0]+m2[0][0], m1[0][1]+m2[0][1], m1[0][2]+m2[0][2]],
            [m1[1][0]+m2[1][0], m1[1][1]+m2[1][1], m1[1][2]+m2[1][2]],
            [m1[2][0]+m2[2][0], m1[2][1]+m2[2][1], m1[2][2]+m2[2][2]]]
            # adds two matrices together

    def find_J_est(self, J_prev, dq, dx):
        # kappa = 1000
        # kappa = 0.001
        # kappa = 0.00000000000001
        # kappa = 1.007
        kappa = 0.01007
        num = self.mat_mult_vec(J_prev, dq)
        num = self.diff(dx, num)
        num = self.vec_mult_mat(dq, num)
        den = self.vec_mult_val(dq, dq)
        frac = self.mat_div_val(num, den)
        frac = self.mat_mult_val(frac, kappa)
        J_est = self.mat_add(frac, J_prev)
        J_est = np.array([J_est[0], J_est[1], J_est[2]])
        return J_est
        # calculates the estimated jacobian, using the above funtions
################################################################################ CALLBACKS
    def q_pos_callback(self, curr_pos):
        self.curr_q = [curr_pos.x, curr_pos.y, curr_pos.z]
        # self.curr_q = self.catch_failed_callback(self.curr_q)
        return self.curr_q
        # the position (q) callback function, returning the current actuator
        # positions

    def pos_callback(self, aurData):
        # self.curr_x = [aurData.x, aurData.y, aurData.z]
        # print(self.jac2pub)
        if self.prev_q == []:
            self.prev_q = [0.1, 0.1, 0.1]
        if self.prev_x == []:
            self.prev_x = [0.1, 0.1, 0.1]
        if self.prev_J == []:
            self.prev_J = self.J_analytical
        # these basically just set a value to the previous q, x and J if it is
        # empty - to avoid an error
        if self.iter == 1:
            self.iter += 1
            self.prev_x = [aurData.x, aurData.y, aurData.z]
            # self.prev_x = self.catch_failed_callback(self.curr_x)
            self.prev_q = self.curr_q
            if self.what_J == 1:
                self.prev_J = self.J_eye
            elif self.what_J == 2 or self.what_J == 3:
                self.prev_J = self.J_analytical
            else:
                print('INVALID INPUT FOR CHOICE OF J')
        # at the first iteration, set x, q and J to the analytical solution
        elif self.iter > 1:
            dx = self.diff(self.prev_x, [aurData.x, aurData.y, aurData.z])
            dq = self.diff(self.prev_q, self.curr_q)
            #calculate dx and dq
            self.prev_x = [aurData.x, aurData.y, aurData.z]
            self.prev_q = self.curr_q
            # set the previous values for the next loop
            if self.what_J == 1:
                J_est = self.find_J_est(self.prev_J, dq, dx)
                self.prev_J = J_est
            elif self.what_J == 2:
                J_est = self.find_J_est(self.J_analytical, dq, dx)
                self.prev_J = J_est
            elif self.what_J == 3:
                J_est = self.prev_J
            # define J based on what is being calculated (position, analytical
            # or estimation)
        self.error = self.diff(self.goal_position, self.prev_x)
        self.err.x = self.error[0]
        self.err.y = self.error[1]
        self.err.z = self.error[2]
        self.pubError.publish(self.err)
        # calculate the error values, and publish  these
        if self.what_J == 1:
            # dq_est = self.mat_mult_vec( inv(self.prev_J), self.error )
            kp = 10000000
            # damping value
            dq_est = self.mat_mult_vec( inv(self.prev_J), ([self.error[0]*-kp, self.error[1]*kp, self.error[2]*kp]) )
        elif self.what_J == 2:
            kp = 0.00001
            kp = 0.1
            # damping value
            dq_est = self.mat_mult_vec( self.prev_J, ([self.error[0]*-kp, self.error[1]*kp, self.error[2]*kp]) )
        elif self.what_J == 3:
            kp = 0.00001
            # damping value
            dq_est = self.mat_mult_vec( self.J_analytical, ([self.error[0]*-kp, self.error[1]*kp, self.error[2]*kp]))
            dq_est[0] = dq_est[0]
            dq_est[1] = dq_est[1]
            dq_est[2] = dq_est[2]
            # I can't remember what these were  for - probably could be removed?
        # this if statement checks which is being calculated, i.e. position or J
        # basically just finds an estimate for the next dq
        self.vel.x = -dq_est[0]
        self.vel.y = -dq_est[1]
        self.vel.z = -dq_est[2]
        # set the velocities to the estimated change in velocity
        self.jac2pub.data = [
            [self.prev_J[0][0], self.prev_J[0][1], self.prev_J[0][2]],
            [self.prev_J[1][0], self.prev_J[1][1], self.prev_J[1][2]],
            [self.prev_J[2][0], self.prev_J[2][1], self.prev_J[2][2]]]
        print(J_est)
        # self.jac2pub.data = self.prev_J
        # print(self.jac2pub.data)
        # self.pubJac.publish(self.jac2pub)
        self.pubVel.publish(self.vel)
        # publish the velocities
################################################################################ MAIN
    def main(self):
        rospy.Subscriber('/curr_pos', Point32, self.q_pos_callback)
        rospy.Subscriber('/final_aurora', Point32, self.pos_callback)
        rospy.spin()
        # subscribes to the position and transformed aurora position data. then
        # repeats the callback function continuously
################################################################################
print('...')
print('Subscribing to /final_aurora ...')
print('...')
print('Subscribing to /curr_pos ...')
print('...')
print('Publishing errors to /Error ...')
print('...')
print('Publishing velocities to /cmd_vel')
print('...')
Jacobian_estimation().main()

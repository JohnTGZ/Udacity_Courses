# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        self.dim_state = params.dim_state
        self.dt = params.dt
        self.q = params.q


    def F(self):
        #State transition matrix
        ############
        # TODO Step 1: implement and return system matrix F
        ############

        return np.matrix( [ 
            [1, 0, 0, self.dt,  0, 0],
            [0, 1, 0, 0, self.dt, 0],
            [0, 0, 1, 0, 0, self.dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]] )
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############
        
        q = self.q
        q1 = (1/2) * self.q * (self.dt)**2
        q2 =  (1/3) * self.q * (self.dt)**3

        return np.matrix(
            [[q2,   0,      0,      q1,     0,      0], 
            [0,     q2,     0,      0,      q1,     0], 
            [0,     0,      q2,     0,      0,      q1], 
            [q1,    0,      0,      q,      0,      0], 
            [0,     q1,     0,      0,      q,      0], 
            [0,     0,      q1,     0,      0,      q]]
        )

        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############
        print("PREDICTION")

        x = track.x
        P = track.P

        x = self.F() * x #State prediction

        P = self.F() * P * self.F().transpose() + self.Q() #Covariance prediction

        track.set_x(x)
        track.set_P(P)

        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############

        x = track.x
        P = track.P
        H = meas.sensor.get_H(x)
        S_mat = self.S(track, meas, H)
        gamma_mat = self.gamma(track, meas)

        K = track.P * H.transpose() * np.linalg.inv(S_mat)
        
        x = x + K * gamma_mat

        P = (np.identity(6) - K * H) * track.P

        track.set_x(x)
        track.set_P(P)

        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############
        gamma = meas.z - meas.sensor.get_hx(track.x)

        return gamma
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############
        S = H * track.P * H.transpose() + meas.R

        return S
        
        ############
        # END student code
        ############ 
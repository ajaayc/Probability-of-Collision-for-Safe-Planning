# -*- coding: utf-8 -*-
#Implements the algorithm to find the probability of collision from the paper "Estimating Probability of Collision for Safe Planning under Gaussian Motion and Sensing Uncertainty"
import numpy as np

def roundAngle(t):
    return t % (2 * np.pi)
def deg2rad(t):
    return t * (np.pi/ 180.0)

class Gauss_Prop():
    def __init__(self):
        self.initParams()

    def initParams(self):
        self.alphas = []
        # Motion noise (in odometry space, see Table 5.5, p.134 in book).
        # variance of noise proportional to alphas
        self.alphas = np.square(np.array([0.05,0.001,0.05,0.01]))

        # Standard deviation of Gaussian sensor noise (independent of distance)
        self.beta = deg2rad(20);
        self.Q = self.beta

    def generateV_EKF(self,prevMu,motioncmd):
        drot1 = motioncmd[0]
        dtrans = motioncmd[1]
        drot2 = motioncmd[2]
        
        prevTheta = prevMu[2];

        V = np.identity(3);
        V[2][0] = 1;
        V[0][0] = -dtrans *  np.sin(prevTheta + drot1);
        V[0][1] = np.cos(prevTheta + drot1);
        V[1][0] = dtrans * np.cos(prevTheta + drot1);
        V[1][1] = np.sin(prevTheta + drot1);

        return V

    def generateM(self,motioncmd):
        drot1 = motioncmd[0]
        dtrans = motioncmd[1]
        drot2 = motioncmd[2]

        alphas1 = self.alphas[0]
        alphas2 = self.alphas[1]
        alphas3 = self.alphas[2]
        alphas4 = self.alphas[3]

        M = np.zeros((3,3))
        M[0][0] = alphas1 * np.square(drot1) + alphas2 * np.square(dtrans);
        M[1][1] = alphas3 * np.square(dtrans) + alphas4 * np.square(drot1) + alphas4 * np.square(drot2);
        M[2][2] = alphas1 * np.square(drot2) + alphas2 * np.square(dtrans);

        #M = -M
        return M

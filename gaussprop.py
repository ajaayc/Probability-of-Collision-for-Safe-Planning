# -*- coding: utf-8 -*-
#Implements the algorithm to find the probability of collision from the paper "Estimating Probability of Collision for Safe Planning under Gaussian Motion and Sensing Uncertainty"
import numpy as np
import linConstraint as lc
from numpy import linalg as LA

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

        # Variance of Gaussian sensor noise (distance to landmark)
        self.Q = np.square(0.5)

        #list of landmarks, i.e. their x,y locations
        self.landmarks = np.array([[1,1],
                                   [2,2]]).transpose()
        self.numlandmarks = np.shape(self.landmarks)[0]
        self.landmarkids = range(self.numlandmarks)

    #Applies sensor model based on given landmark id
    def observation(self,state,landmarkid):
        currlmk = self.landmarks[:][landmarkid]
        print 'currlmk:', currlmk
        
        #Get distance
        s = state[0:2]
        diff = s - currlmk
        distance = LA.norm(diff,axis=0)
        return distance

    #Samples a 1 variable Gaussian with mean and sigma
    def sample(self,mean,sigma):
        num = sigma * np.random.randn(1,1) + mean
        return num
        
    
    #Generates pose following a noisy control input
    def sampleOdometry(self,state,motioncmd):
        drot1 = motioncmd[0]
        dtrans = motioncmd[1]
        drot2 = motioncmd[2]

        alphas1 = self.alphas[0]
        alphas2 = self.alphas[1]
        alphas3 = self.alphas[2]
        alphas4 = self.alphas[3]

        noisymotion = np.zeros(np.shape(motioncmd))
        
        noisymotion[0] = self.sample(drot1,alphas1*np.square(drot1)+alphas2*np.square(dtrans));
        noisymotion[1] = self.sample(dtrans,alphas3*np.square(dtrans)+alphas4*(np.square(drot1)+np.square(drot2)));
        noisymotion[2] = self.sample(drot2,alphas1*np.square(drot2)+alphas2*np.square(dtrans));

        newstate = self.prediction(state, noisymotion);

        return newstate
    
    #Returns list of measurements to all landmarks from sensor from the state, corrupted by random Gaussian noise
    def sensorReading(self,state):
        #For each landmark, find distance to it from x
        loc = state[0:2]
        loc = np.reshape(loc,[2,1])
        print "loc:", loc

        print 'landmarks:', self.landmarks
        diff = self.landmarks - loc
        print "diff:", diff
        model = LA.norm(diff,axis=0)

        #Add 0-mean Gaussian noise to each
        noises = np.sqrt(self.Q) * np.random.randn(1,self.numlandmarks) + 0

        print "Model:", model
        print "noises:", noises
        measured = model + noises
        return measured
        
    #Applies odometry motion model
    def prediction(self,state,motioncmd):
        drot1 = motioncmd[0]
        dtrans = motioncmd[1]
        drot2 = motioncmd[2]

        x = state[0]
        y = state[1]
        theta = state[2]
        
        newstate = np.zeros(np.shape(state))

        newstate[0] = x + dtrans * np.cos(theta + drot1)
        newstate[1] = y + dtrans * np.sin(theta + drot1)
        newstate[2] = theta + drot1 + drot2

        newstate[2] = roundAngle(newstate[2])
        
        return newstate

    #Given two poses, compute the odometry command between them
    def inverseOdometry(self,p1,p2):
        drot1 = np.arctan2(p2[1] - p1[1], p2[0] - p1[0]) - p1[2]
        drot1 = roundAngle(drot1)

        dtrans = np.sqrt(np.square(p2[0] - p1[0]) + np.square(p2[1] - p1[1]))

        drot2 = p2[2] - p1[2] - drot1
        drot2 = roundAngle(drot2)

        return np.array([drot1,dtrans,drot2])

    #Given a list of the states for the motion plan, determines the list of
    #odometry commands needed to move from one state to the next
    def getPathOdometry(self,path):
        ulist = []
        for t in range(len(path) - 1):
            #Get odometry between x_t and x_t+1
            u = self.inverseOdometry(path[t],path[t+1])
            ulist.append(u)
        return ulist

    #Jacobian of motion model with respect to control input
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

    #Odometry noise
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

    #Jacobian of motion model with respect to state.
    def generateG_EKF(self,prevMu,motioncmd):
        drot1 = motioncmd[0]
        dtrans = motioncmd[1]
        drot2 = motioncmd[2]

        prevTheta = prevMu[2];

        G = np.identity(3);
        G[0][2] = -dtrans * np.sin(prevTheta + drot1);
        G[1][2] = dtrans * np.cos(prevTheta + drot1);

        return G

# -*- coding: utf-8 -*-
#Implements the algorithm to find the probability of collision from the paper "Estimating Probability of Collision for Safe Planning under Gaussian Motion and Sensing Uncertainty"
import numpy as np
import linConstraint as lc
from numpy import linalg as LA
#from numpy.linalg import inv

def roundAngle(t):
    return t % (2 * np.pi)
def deg2rad(t):
    return t * (np.pi/ 180.0)

#Samples a 1 variable Gaussian with mean and sigma
def sample(mean,sigma):
    num = sigma * np.random.randn(1,1) + mean
    return num


class Gauss_Prop():
    def __init__(self,env,robot,room):
        self.env = env
        self.robot = robot
        self.room = room

        #Drawing handles
        self.handles = []

        self.initParams()
        self.initConstraints()
        #self.drawBeacons()

    def initParams(self):
        self.alphas = []
        # Motion noise (in odometry space, see Table 5.5, p.134 in book).
        # variance of noise proportional to alphas
        self.alphas = np.square(np.array([0.05,0.001,0.05,0.01]))

        # Variance of Gaussian sensor noise (distance to landmark)
        self.Q_noise = np.square(0.5)
        self.Q = self.Q_noise * np.identity(2)

        #list of landmarks, i.e. their x,y locations
        self.landmarks = np.array([[3,-3],
                                   [0, 0]])
        self.numlandmarks = np.shape(self.landmarks)[1]
        self.landmarkids = range(self.numlandmarks)

        #initialized 
        self.constraints = []

        #Initial robot position and covariance

    #Initializes linear space constraints
    def initConstraints(self):
        
        pass

    #Returns True if there's a collision
    def linearCollisionCheck(self,state):
        #Loop through all constraints
        for c in self.constraints:
            result = c.checkConstraint(state)
            #If failed to pass constraint, return true for a collision occurring
            if result == False:
                return True

        #No collision occurred
        return False
        
    #Applies sensor model based on given landmark id
    def observation(self,state,landmarkid):
        currlmk = self.landmarks[:][landmarkid]
        print 'currlmk:', currlmk
        
        #Get distance
        s = state[0:2]
        diff = s - currlmk
        distance = LA.norm(diff,axis=0)
        return distance        
    
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
        
        noisymotion[0] = sample(drot1,alphas1*np.square(drot1)+alphas2*np.square(dtrans));
        noisymotion[1] = sample(dtrans,alphas3*np.square(dtrans)+alphas4*(np.square(drot1)+np.square(drot2)));
        noisymotion[2] = sample(drot2,alphas1*np.square(drot2)+alphas2*np.square(dtrans));

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
    #Same as V from Thrun book
    def generateB(self,prevMu,motioncmd):
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

    #Jacobian of 3x1 motion model with respect to 3x1 Gaussian noise variables
    def generateV(self):
        return np.identity(3)

    #Jacobian of 2x1 sensor model with respect to 2x1 Gaussian noise variables
    def generateW(self):
        return np.identity(2)

    #Jacobian of 2x1 sensor model with respect to state.
    #Same notation as Thrun book
    #TODO: Extend this for case of more than 2 landmarks
    def generateH(self,state):
        #Make two HRows
        r1 = self.makeHRow(state,0)
        r2 = self.makeHRow(state,1)

        #Stack them
        H = np.array([r1,r2])
        return H

    def makeHRow(self,state,landmarkid):
        mx = self.landmarks[0,landmarkid]
        my = self.landmarks[1,landmarkid]

        x = state[0]
        y = state[1]

        diff = np.array([x,y]) - np.array([mx,my])
        q = np.square(diff[0]) + np.square(diff[1])
        
        entry1 = -(mx - x)/np.sqrt(q)
        entry2 = -(my - y)/np.sqrt(q)
        entry3 = 0

        return [entry1,entry2,entry3]
    
    #Odometry noise
    def generateM_EKF(self,motioncmd):
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
    #Same as G in Thrun book
    def generateA(self,prevMu,motioncmd):
        drot1 = motioncmd[0]
        dtrans = motioncmd[1]
        drot2 = motioncmd[2]

        prevTheta = prevMu[2];

        G = np.identity(3);
        G[0][2] = -dtrans * np.sin(prevTheta + drot1);
        G[1][2] = dtrans * np.cos(prevTheta + drot1);

        return G

    def getKalmanGain(self,nominalx,nominalu,deviationPrevMu,deviationPrevSigma,deviationcontrol):
        #Apply linearized motion model to predict next state deviation
        A = self.generateA(nominalx,nominalu)
        B = self.generateB(nominalx,nominalu)
        V = self.generateV()

        #Now apply the matrices
        mubardeviation = A * deviationPrevMu + B * deviationcontrol

        #Transform sigma
        sigmabardeviation = A * deviationPrevSigma * A.transpose()

        #Add R, Thrun Book
        M = self.generateM_EKF(nominalu)
        #TODO: Is this the correct M?
        sigmabardeviation = sigmabardeviation + B * M * B.transpose()

        H = self.generateH(nominalx)
        Q = self.Q
        
        K = sigmabardeviation * H.transpose() * LA.inv(H * sigmabardeviation * H.transpose() + Q)
        return K
        
    #Draws green beacons in OpenRave
    def drawBeacons(self):
        #Green beacons
        pcolors = np.array(((56/255.0,249/255.0,26/255.0,1)))

        #Loop through all landmarks
        for l in self.landmarkids:
            currbeacon = self.landmarks[:,l]

            with self.env:
                self.handles.append(self.env.plot3(points=np.array((currbeacon[0],currbeacon[1],1)),
                    pointsize=0.2,
                    colors=pcolors,
                    drawstyle = 2

                ))

    #------------------------------------------------------------
    #Stuff below this is for the actual paper
    #------------------------------------------------------------
    def estimateCollision():

        pass


    #Compute the 3x3 control gain matrix L_t+1
    def generateL(self,nominalcurrstate,estimatedcurrstate,nominalgoalstate,nominalcontrol):
        #Get (estimate) of the state deviation
        xhatt = estimatedcurrstate - nominalcurrstate

        #Get odometry needed to move from estimated currstate to nominalgoalstate
        urequired = self.inverseOdometry(estimatedcurrstate,nominalgoalstate)

        #Get difference between u and u*
        ubar = urequired - nominalcontrol

        #Find the 3x3 linear transformation L needed to move from xhatt to ubar
        L = np.identity(3)

        #TODO: Think about this divide by 0. This would occur if xhatt
        #has 0's. i.e. deviation between curr state and nominal state is 0
        L[0][0] = ubar[0] / (float(xhatt[0]) if xhatt[0] != 0 else 0.1)
        L[1][1] = ubar[1] / (float(xhatt[1]) if xhatt[1] != 0 else 0.1)
        L[2][2] = ubar[2] / (float(xhatt[2]) if xhatt[2] != 0 else 0.1)

        return L

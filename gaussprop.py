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
        self.alphas = np.square(np.array([0.1, 0.001, 0.05, 0.01]))

        # Variance of Gaussian sensor noise (distance to landmark)
        self.Q_noise = np.square(0.5)
        self.Q = self.Q_noise

        #list of landmarks, i.e. their x,y locations
        #self.landmarks = np.array([[3,-3,0, 0, 0,  0,-1,1, 0,-1,1,   -1, 1],
        #                           [0, 0,3,-3, 1, -1, 0,0,-1, 1,1,   -1,-1]])
        self.landmarks = np.array([[3,-3,0, 0],
                                   [0, 0,3,-3]])

        self.numlandmarks = np.shape(self.landmarks)[1]
        self.landmarkids = range(self.numlandmarks)

        #initialized 
        self.constraints = []

        #Initial robot position and covariance
        self.initialStateMean = []
        self.initialStateCovariance = []
        
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
        currlmk = self.landmarks[:,landmarkid]
        #print 'currlmk:', currlmk
        
        #Get distance
        s = state[0:2]
        diff = s - currlmk
        distance = LA.norm(diff,axis=0)
        return distance        

    def sampleObservation(self,state,landmarkid):
        distance = self.observation(state,landmarkid)
        #Add Gaussian noise to distance
        return (distance + sample(0,np.sqrt(self.Q)))
    
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

        return np.array([entry1,entry2,entry3])
    
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
    def generateG_EKF(self,prevMu,motioncmd):
        drot1 = motioncmd[0]
        dtrans = motioncmd[1]
        drot2 = motioncmd[2]

        prevTheta = prevMu[2];

        G = np.identity(3);
        G[0][2] = -dtrans * np.sin(prevTheta + drot1);
        G[1][2] = dtrans * np.cos(prevTheta + drot1);

        return G
        
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

    #Compute the 3x3 control gain matrix L_t+1
    def generateL(self,nominalcurrstate,estimatedcurrstate,nominalgoalstate,nominalcontrol):
        #Get (estimate) of the state deviation
        xhatt = np.array(estimatedcurrstate) - np.array(nominalcurrstate)

        #Get odometry needed to move from estimated currstate to nominalgoalstate
        urequired = self.inverseOdometry(estimatedcurrstate,nominalgoalstate)

        #Get difference between u and u*
        ubar = np.array(urequired) - np.array(nominalcontrol)
        print 'ubar: ', ubar

        #Find the 3x3 linear transformation L needed to move from xhatt to ubar
        L = np.identity(3)

        #TODO: Think about this divide by 0. This would occur if xhatt
        #has 0's. i.e. deviation between curr state and nominal state is 0
        L[0][0] = ubar[0] / (float(xhatt[0]) if xhatt[0] != 0 else 0.1)
        L[1][1] = ubar[1] / (float(xhatt[1]) if xhatt[1] != 0 else 0.1)
        L[2][2] = ubar[2] / (float(xhatt[2]) if xhatt[2] != 0 else 0.1)

        return L


    #------------------------------------------------------------
    #Stuff below this is for the actual paper
    #------------------------------------------------------------
    #trajectory is list of states for the motion plan
    #controlinputs is list of odometry commands to transition between states
    #len(controls) = len(trajectory) - 1
    def EKF_GaussProp(self,trajectory,controlinputs):
        self.initialStateMean = trajectory[0]
        self.initialStateCovariance = .001 * np.identity(3)

        #Initialize mean and covariance
        mu = self.initialStateMean;
        cov = self.initialStateCovariance;

        #Store the real state (we don't know this in practice)
        realstate = trajectory[0]
        
        #simulate trajectory
        for i,control in enumerate(controlinputs):
            #Get motion command
            motionCommand = controlinputs[i]

            M = self.generateM_EKF(motionCommand);
            Q = self.Q

            #Get control gain to move to next state
            nominalstate = trajectory[i]
            estimatedstate = mu
            nominalgoal = trajectory[i+1]
            nominalcontrol = controlinputs[i]

            gain = self.generateL(nominalstate,estimatedstate,nominalgoal,nominalcontrol)
            #Multiply gain by deviation in state to get deviation to add to u*
            statedeviation = np.array(estimatedstate) - np.array(nominalstate)
            statedeviation = np.asmatrix(statedeviation).transpose()

            controldeviation = gain * statedeviation
            controldeviation = controldeviation.transpose()

            #Add control deviation to nominal control
            appliedcontrol = np.array(nominalcontrol) + np.array(controldeviation)
            appliedcontrol = appliedcontrol[0]

            #------------------------------------------------------------
            #EKF Predict. Predict where we'll go based on applied control
            predMu,predSigma = self.EKFpredict(mu,cov,appliedcontrol,M,Q)
            #------------------------------------------------------------

            #Now move (with noise)
            #Add noise to odometry to go to another state
            nextstate = self.sampleOdometry(realstate,appliedcontrol)
            realstate = nextstate
            print 'realstate: ', realstate

            realobservations = np.zeros([1,self.numlandmarks])
            
            #Get sensor measurements from the real state. Loop
            #through all landmarks
            for currlid in range(self.numlandmarks):
                z = self.sampleObservation(realstate,currlid)
                realobservations[0,currlid] = z[0][0]
            
            #------------------------------------------------------------
            #EKF Update of estimated state and covariance based on the measurements

            #import pdb
            #pdb.set_trace()

            realobservations = list(realobservations[0])
            newmu,newsigma = self.EKFupdate(predMu,predSigma,realobservations,Q)
            mu = newmu
            cov = newsigma

            print 'nominalstate: ', trajectory[i+1]
            print 'estimatestate: ', mu
            print 'estimatecov: ', cov
            
            #
            #------------------------------------------------------------
            

            
    def EKFpredict(self,mu,Sigma,u,M,Q):
        #Get G matrix and V matrix
        G = self.generateG_EKF(mu,u)
        V = self.generateV_EKF(mu,u)

        #noise in odometry
        R = V * M * V.transpose()

        predMu = self.prediction(mu,u)
        predSigma = G * Sigma * G.transpose() + R

        return predMu,predSigma

    def EKFupdate(self,predMu,predSigma,measurements,Q):
        #Loop through all measurements
        for lid,measurement in enumerate(measurements):
            #lid is landmark id. measurement is the distance
            #to the landmark recorded by sensor
            landmark_x = self.landmarks[0,lid]
            landmark_y = self.landmarks[1,lid]

            # Lines 10-13 of EKF Algorithm
            H = self.makeHRow(predMu,lid)
            H = np.asmatrix(H)

            #Innovation / residual covariance

            S = H * predSigma * H.transpose() + Q;

            # Kalman gain
            K = predSigma * H.transpose() * LA.inv(S)

            #z and zhat
            z = measurement
            zhat = self.observation(predMu,lid)
            
            # Correction
            temp = z - zhat;
            predMu = np.asmatrix(predMu).transpose() + K * (temp);
            #Make predMu an array again
            predMu = predMu.transpose()
            predMu = np.array(predMu)[0]
            predSigma = (np.identity(3) - K * H) * predSigma;
            

        return predMu,predSigma

            
    # def getKalmanGain(self,nominalx,nominalu,deviationPrevMu,deviationPrevSigma,deviationcontrol):
    #     #Apply linearized motion model to predict next state deviation
    #     A = self.generateA(nominalx,nominalu)
    #     B = self.generateB(nominalx,nominalu)
    #     V = self.generateV()

    #     #Now apply the matrices
    #     mubardeviation = A * deviationPrevMu + B * deviationcontrol

    #     #Transform sigma
    #     sigmabardeviation = A * deviationPrevSigma * A.transpose()

    #     #Add R, Thrun Book
    #     M = self.generateM_EKF(nominalu)
    #     #TODO: Is this the correct M?
    #     sigmabardeviation = sigmabardeviation + B * M * B.transpose()

    #     H = self.generateH(nominalx)
    #     Q = self.Q
        
    #     K = sigmabardeviation * H.transpose() * LA.inv(H * sigmabardeviation * H.transpose() + Q)
    #     return K

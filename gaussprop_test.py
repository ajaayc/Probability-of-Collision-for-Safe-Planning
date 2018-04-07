import gaussprop as gp
import numpy as np
import cPickle as pickle

prop = gp.Gauss_Prop(1,2,3)
#prop.generateM_EKF([1,2,3])
#prop.generateV_EKF([1,2,3],[1,2,3])
#prop.generateG_EKF([1,2,3],[1,2,3])
#prop.prediction([1,2,3],[1,2,3])
#print prop.sensorReading(np.array([1,2,3]))
#print prop.observation(np.array([8,2,3]),0)
#prop.sampleOdometry([1,2,3],[1,2,3])
#print prop.inverseOdometry([2,2,0],[3,3,0])
#
#nominalcurrstate = np.array([1,1,np.pi/2])
#estimatedcurrstate = np.array([1,1.41,np.pi/2])
#nominalgoalstate = np.array([2,1,np.pi/2])
#nominalcontrol = prop.inverseOdometry(nominalcurrstate,nominalgoalstate)
#
#Lmatrix = prop.generateL(nominalcurrstate,estimatedcurrstate,nominalgoalstate,nominalcontrol)
#print Lmatrix
#
#print 'H matrix: ',prop.generateH([1,2,0])

#K = prop.getKalmanGain([0,1,0],[0.1,0.1,0.1],[0,0.1,0],np.identity(3),[0.1,0.1,0.1])
#print K

#Load trajectory and odometry
file_temp = open('trajectory.dat','rb')
trajectory = pickle.load(file_temp)
file_temp.close()

file_temp = open('odometry.dat','rb')
odometry = pickle.load(file_temp)
file_temp.close()

prop.EKF_GaussProp(trajectory,odometry)

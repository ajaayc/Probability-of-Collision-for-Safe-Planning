#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import os
import openravepy
import numpy as np
import sys
import traceback
import gaussprop as gp
import datetime

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def writeReport(numSimulations,envfile,alphas,Q,numlandmarks,landmarks,numparticles,initialcovariance,trajectory,odometry,simulationTimes,collisionProportions,simTimeAverage,collisionPropAverage):
    print "Writing Report\n"
    ts = time.time()
    st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H_%M_%S')
    #print st
    fname = 'simReport_' + st +  '.txt'
    
    f = open(fname,'w')
    f.write('Environment: ' + str(envfile) + "\n")
    f.write('Num Landmarks: ' + str(numlandmarks) + "\n")
    f.write('Landmarks: \n' + str(landmarks) + "\n")

    f.write('Alphas: \n' + str(alphas) + "\n")
    f.write('Sensor Noise Variance: ' + str(Q) + "\n")
    f.write('Initial Covariance: \n' + str(initialcovariance) + "\n")
    f.write('---------------------------------\n')
    f.write('NumSimulations: ' + str(numSimulations) + "\n")
    f.write('Num Particles: ' + str(numparticles) + "\n")

    f.write('Simulation Times: \n' + str(simulationTimes) + "\n")
    f.write('Collision Proportions: \n' + str(collisionProportions) + "\n")
    f.write('Average Sim Time: ' + str(simTimeAverage) + "\n")
    f.write('Average Prob Collision: ' + str(collisionPropAverage) + "\n")
    f.write('---------------------------------\n')
    f.write('Trajectory: \n' + str(trajectory.transpose()) + "\n")
    f.write('Odometry: \n' + str(odometry.transpose()) + "\n")

    f.close()
    

def writeReportGMM(numSimulations,envfile,alphas,Q,numlandmarks,landmarks,numparticles,initialcovariance,trajectory,odometry,simulationTimes,collisionProportions,simTimeAverage,collisionPropAverage,numGaussians,numSamples):
    print "Writing Report\n"
    ts = time.time()
    st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H_%M_%S')
    #print st
    fname = 'GMMsimReport_' + st +  '.txt'#DIFFERENT
    
    f = open(fname,'w')
    f.write('Environment: ' + str(envfile) + "\n")
    f.write('Num Landmarks: ' + str(numlandmarks) + "\n")
    f.write('Landmarks: \n' + str(landmarks) + "\n")

    f.write('Alphas: \n' + str(alphas) + "\n")
    f.write('Sensor Noise Variance: ' + str(Q) + "\n")
    f.write('Initial Covariance: \n' + str(initialcovariance) + "\n")
    f.write('---------------------------------\n')
    f.write('NumSimulations: ' + str(numSimulations) + "\n")
    #f.write('Num Particles: ' + str(numparticles) + "\n")

    f.write('Num Samples: ' + str(numSamples) + "\n")#DIFFERENT
    f.write('Num Gaussians: ' + str(numGaussians) + "\n")#DIFFERENT
    
    
    f.write('Simulation Times: \n' + str(simulationTimes) + "\n")
    f.write('Collision Proportions: \n' + str(collisionProportions) + "\n")
    f.write('Average Sim Time: ' + str(simTimeAverage) + "\n")
    f.write('Average Prob Collision: ' + str(collisionPropAverage) + "\n")
    f.write('---------------------------------\n')
    f.write('Trajectory: \n' + str(trajectory.transpose()) + "\n")
    f.write('Odometry: \n' + str(odometry.transpose()) + "\n")

    f.close()


#For sending configuration to C++
def list2String(mylist):
    result = ""
    for val in mylist:
        result = result + str(val) + " "
    return result

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

if __name__ == "__main__":
    usage = 'Usage: ' + sys.argv[0] + ' \"MC\" or \"GMM\"'
    if len(sys.argv) != 2:
        #Incorrect format
        print "Incorrect number of arguments"
        print usage
        sys.exit()

    if not(sys.argv[1] =='MC' or sys.argv[1] == 'GMM'):
        print "Unrecognized option"
        print usage
        sys.exit()

    simoption = sys.argv[1]

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()        
    # load a scene from ProjectRoom environment XML file
    #env.Load('pr2custom.env.xml')
    envfile = 'data/pr2test2.env.xml'
    env.Load(envfile)
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    ### INITIALIZE YOUR PLUGIN HERE ###
    #prob = RaveCreateModule(env,'MyModule')
    RaveLoadPlugin('mcsimplugin/build/mcsimplugin')
    MCModule = RaveCreateModule(env,'MCModule')
    env.AddModule(MCModule,args='')
    print MCModule.SendCommand('help')
    print MCModule.SendCommand('MyCommand [0.1,4.5,7.5,4.7]')

    ### END INITIALIZING YOUR PLUGIN ###

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);

    with env:
        # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])

    room = env.GetBodies()[0]

    #Initialize gaussian propagation object
    prop = gp.Gauss_Prop(env,robot,room)
    prop.drawBeacons()

    #Send alphas, Q, and beacons to C++
    print "Sending alphas to C++:"
    alphas = list2String(list(prop.alphas))
    MCModule.SendCommand('setAlphas ' + alphas)
    Q = prop.Q
    MCModule.SendCommand('setQ ' + str(Q))

    MCModule.SendCommand('setNumLandmarks ' + str(prop.numlandmarks))
    MCModule.SendCommand('setLandmarks ' + list2String(list(prop.landmarks[0,:])) + list2String(list(prop.landmarks[1,:])))

    numParticles = 10000
    MCModule.SendCommand('setNumParticles ' + str(numParticles))
    

    #Send initial covariance matrix
    strcov = list2String(list(prop.initialStateCovariance[0,:])) + list2String(list(prop.initialStateCovariance[1,:])) + list2String(list(prop.initialStateCovariance[2,:]))
    MCModule.SendCommand('setInitialCovariance ' + strcov)


    
    # Send motion plan and odometry to C++
    #Load trajectory and odometry
    file_temp = open('trajectory.dat','rb')
    trajectory = pickle.load(file_temp)
    file_temp.close()

    file_temp = open('odometry.dat','rb')
    odometry = pickle.load(file_temp)
    file_temp.close()

    pathlen = len(trajectory)
    
    MCModule.SendCommand('setPathLength ' + str(pathlen))

    trajectory = np.array(trajectory).transpose()
    odometry = np.array(odometry).transpose()

    #import pdb
    #pdb.set_trace()

    #Send trajectory
    strtraj = list2String((trajectory[0,:]).tolist()) + list2String((trajectory[1,:]).tolist()) + list2String((trajectory[2,:]).tolist())
    strodometry = list2String((odometry[0,:]).tolist()) + list2String((odometry[1,:]).tolist()) + list2String((odometry[2,:]).tolist())
    MCModule.SendCommand('setTrajectory ' + strtraj)
    MCModule.SendCommand('setOdometry ' + strodometry)


    #--------------------
    #GMM Stuff
    if simoption == "GMM":
        numGaussians = 2
        numGMMSamples = numParticles
        MCModule.SendCommand('setNumGaussians ' + str(numGaussians))
        MCModule.SendCommand('setNumGMMSamples ' + str(numGMMSamples))

        #start = time.clock()
        #collprop = MCModule.SendCommand('runGMMEstimation')
        #end = time.clock()
        #collprop = float(collprop)
        #print 'Python got collision proportion: ', collprop
        #simTime = end - start
        #print "Python GMM Time: ", simTime

    #--------------------
    #Monte Carlo Simulation Stuff
    #elif simoption == "MC":

    numSimulations = 200

    simTimes = []
    proportions = []

    #Use this file to store times and proportions if simulation is stopped in the middle
    ts = time.time()
    st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H_%M_%S')
    #print st

    if simoption == 'MC':
        fname = 'checkpoint_' + st +  '.txt'
    elif simoption == 'GMM':
        fname = 'GMMcheckpoint_' + st +  '.txt'
        
    f2 = open(fname,'w')

    for i in range(numSimulations):
        start = time.clock()
        if simoption == "MC":
            collprop = MCModule.SendCommand('runSimulation')
        elif simoption == "GMM":
            collprop = MCModule.SendCommand('runGMMEstimation')
        end = time.clock()
        collprop = float(collprop)
        print 'Python got collision proportion: ', collprop
        simTime = end - start
        print "Python Simulation Time: ", simTime
        simTimes.append(simTime)
        f2.write('Simulation: ' + str(i) + '\n')
        f2.write('simTime: ' + str(simTime) + '\n')
        proportions.append(collprop)
        f2.write('collProp: ' + str(collprop) + '\n')
        f2.flush()
        os.fsync(f2.fileno())
    f2.close()

    print "SimTimes: \n", simTimes
    print "Collision Probs: \n", proportions

    #Averages
    simTA = np.average(simTimes)
    CPA = np.average(proportions)

    if simoption == "MC":
        #Output stats and configuration to text file
        writeReport(numSimulations,envfile,alphas,Q,prop.numlandmarks,prop.landmarks,numParticles,prop.initialStateCovariance,trajectory,odometry,simTimes,proportions,simTA,CPA);
    elif simoption == "GMM":
        writeReportGMM(numSimulations,envfile,alphas,Q,prop.numlandmarks,prop.landmarks,numParticles,prop.initialStateCovariance,trajectory,odometry,simTimes,proportions,simTA,CPA,numGaussians,numGMMSamples);
    raw_input("Press enter to exit...")


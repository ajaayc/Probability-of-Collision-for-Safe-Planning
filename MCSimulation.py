#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
import numpy as np
import sys
import traceback
import gaussprop as gp

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

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
    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()        
    # load a scene from ProjectRoom environment XML file
    env.Load('pr2custom.env.xml')
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

    room = env.GetBodies()[0]

    #Initialize gaussian propagation object
    prop = gp.Gauss_Prop(env,robot,room)
    prop.drawBeacons()

    #Send alphas, Q, and beacons to C++
    print "Sending alphas to C++:"
    alphas = list2String(list(prop.alphas))
    MCModule.SendCommand('setAlphas ' + alphas)
    MCModule.SendCommand('setQ ' + str(prop.Q))

    import pdb
    pdb.set_trace()
    MCModule.SendCommand('setNumLandmarks ' + str(prop.numlandmarks))
    MCModule.SendCommand('setLandmarks ' + list2String(list(prop.landmarks[0,:])) + list2String(list(prop.landmarks[1,:])))

    numParticles = 1000
    MCModule.SendCommand('setNumParticles ' + str(numParticles))
    
    # #set start config

    # start = time.clock()
    # with env:
    #     goalconfig = [0.449,-0.201,-0.151,0,0,-0.11,0]
    #     ### YOUR CODE HERE ###
    #     ###call your plugin to plan, draw, and execute a path from the current configuration of the left arm to the goalconfig
    #     startstr = list2String(startconfig);
    #     goalstr = list2String(goalconfig);

    #     startstr = 'start ' + startstr
    #     goalstr = 'goal ' + goalstr

    #     print 'startstr from Python:', startstr
    #     print 'goalstr from Python:', goalstr
        
    #     #Tell C++ the start and goal nodes
    #     MCModule.SendCommand('setNode ' + startstr)
    #     MCModule.SendCommand('setNode ' + goalstr)
    #     MCModule.SendCommand('printStartGoal')

    #     goalbias = .05
        
    #     #Tell C++ to initialize an RRTConnect with goal bias
    #     MCModule.SendCommand('initRRT rrt ' + str(goalbias))
        
    #     #Tell C++ to start planning with the RRT
    #     result = MCModule.SendCommand('runRRT')
    #     print "Python got path from C++: ",result
    #     import pdb
    #     pdb.set_trace()
    #     finalpath = convertPathString2Path(result)

    #     handles = []
    #     #Now draw red squares
    #     drawPath(finalpath,handles,robot,env,'red')
        
    # end = time.clock()
    # print "Planning Time: ", end - start

    # #Now execute trajectory
    # traj = ConvertPathToTrajectory(robot, finalpath)
    # # Execute the trajectory on the robot.
    # if traj != None:
    #     robot.GetController().SetPath(traj)
        
    #     ### END OF YOUR CODE ###
    # waitrobot(robot)

    raw_input("Press enter to exit...")


#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
import numpy as np
import sys
import traceback
import gaussprop as gp
import cPickle as pickle

#### YOUR IMPORTS GO HERE ####
import astar_planner as astar

#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def getRotationMat(alph):
    rot = np.array([[np.cos(alph),np.sin(alph),0,0],
                    [-np.sin(alph),np.cos(alph),0,0],
                    [0,0,1,0],
                    [0,0,0,1]])
    return rot

def getTranslationMat(x,y,z):
    trans = np.array([[1,0,0,x],
                        [0,1,0,y],
                        [0,0,1,z],
                        [0,0,0,1]])
    return trans

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


def ConvertPathToTrajectory(robot,path=[]):
#Path should be of the form path = [q_1, q_2, q_3,...], where q_i = [x_i, y_i, theta_i]

    if not path:
	return None
    # Initialize trajectory
    traj = RaveCreateTrajectory(env,'')	
    traj.Init(robot.GetActiveConfigurationSpecification())
    for i in range(0,len(path)):
	traj.Insert(i,numpy.array(path[i]))
    # Move Robot Through Trajectory
    planningutils.RetimeAffineTrajectory(traj,maxvelocities=ones(3),maxaccelerations=5*ones(3))
    return traj


if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    environments = ['pr2custom.env.xml','data/pr2test2.env.xml']
    goals =        [[2.28,0.11,0],      [2.6,-1.3,-pi/2]]
    #Change this to specify configuration you want
    envindex = 1
    
    env.Reset()
    # load a scene from ProjectRoom environment XML file
    #env.Load('data/pr2test2.env.xml')
    #env.Load('pr2custom.env.xml')
    env.Load(environments[envindex])
    #env.Load('data/ikeatable.kinbody.xml')
    #env.Load('playbox.kinbody.xml')
    time.sleep(0.1)

    #import pdb
    #pdb.set_trace()
    
    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]
    
    # tuck in the PR2's arms for driving
    tuckarms(env,robot);


    with env:
        # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
        #import pdb
        #pdb.set_trace()
        goalconfig = goals[envindex]
    
    #Drawing handles    
    handles = []
    
    start = time.clock()

    #### YOUR CODE HERE ####
    #### Implement your algorithm to compute a path for the robot's base starting 
    #### from the current configuration of the robot and ending at goalconfig.
    #### The robot's base DOF have already been set as active. It may be easier
    #### to implement this as a function in a separate file and call it here.

    #Higher distance weight penalizes farther distances


    #---------8 Euclidean Successful-----------

    distanceWeight = 100
    #Penalize    
    angleWeight = 5

    distanceDisc = 0.15
    angleDisc = np.pi/2

    room = env.GetBodies()[0]

    prop = gp.Gauss_Prop(env,robot,room)
    prop.drawBeacons()
    #import pdb
    #pdb.set_trace()

    planner = astar.A_Star_Planner_8_euclidean(goalconfig,distanceWeight,angleWeight,distanceDisc,angleDisc,env,robot,room)    

    #---------8 Manhatten Successful-----------

#    distanceWeight = 100
#    #Penalize    
#    angleWeight = 5
#
#    distanceDisc = 0.1
#    angleDisc = np.pi/2
#
#
#    planner = astar.A_Star_Planner_8_manhatten(goalconfig,distanceWeight,angleWeight,distanceDisc,angleDisc,env,robot,room)    

    #---------4 Manhatten Successful-----------

#    distanceWeight = 100
#    #Penalize    
#    angleWeight = 25
#
#    distanceDisc = 0.08
#    angleDisc = np.pi/2
#
#
#    planner = astar.A_Star_Planner_4_manhatten(goalconfig,distanceWeight,angleWeight,distanceDisc,angleDisc,env,robot,room)
    
 #---------4 Euclidean Successful-----------

#    distanceWeight = 100
#    #Penalize    
#    angleWeight = 25
#
#    distanceDisc = 0.08
#    angleDisc = np.pi/2
#
#
#    planner = astar.A_Star_Planner_4_euclidean(goalconfig,distanceWeight,angleWeight,distanceDisc,angleDisc,env,robot,room)

    pathfind = planner.searchPath()
    if pathfind == False:
        print "No Solution Found"
        raw_input("Press enter to exit...")
        sys.exit()
    else:
        print "Path found!!!"
        print "Path Cost: ",planner.getGScore(planner.boxGoal)

    #TODO: Check errors in path finding. Also see if need to check table collisions
    planner.backtrackPath()
    path = planner.getPath()
    
    #### Draw the X and Y components of the configurations explored by your algorithm

    #path = [] #put your final path in this variable
    #Path is a list of configurations. path = [q0,q1,q2,...], where qi=[x,y,theta]

    #### END OF YOUR CODE ###
    end = time.clock()
    print "Time: ", end - start

    #import pdb
    #pdb.set_trace()
    
    ulist = prop.getPathOdometry(path)

    #Save path and odometry to file
    file_temp = open('trajectory.dat', 'w')
    pickle.dump(path, file_temp)
    file_temp.close()

    file_temp = open('odometry.dat', 'w')
    pickle.dump(ulist, file_temp)
    file_temp.close()
    
    
    # Now that you have computed a path, convert it to an openrave trajectory 
    traj = ConvertPathToTrajectory(robot, path)
    
    	# Execute the trajectory on the robot.
    if traj != None:
        robot.GetController().SetPath(traj)
    
    
    waitrobot(robot)
    
    raw_input("Press enter to exit...")


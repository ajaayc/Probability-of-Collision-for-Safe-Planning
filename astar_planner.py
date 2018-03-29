# -*- coding: utf-8 -*-
"""
Implements A Star Algorithm with 4,8 connected neighbors and euclidean,manhatten distance
"""

from abc import ABCMeta, abstractmethod
import math
import numpy as np
import heapq as hq
import array
import sys
import openravepy

class pqueue_entry():
    goalNode = None    
    
    def __init__(self,node,parent,f_score):
        self.node = node
        self.parent = parent
        self.f_score = f_score
    #Returns true if self's priority is less than other's priority
    #smallest f_scores first
    def __cmp__(self,other):
        return self.f_score > other.f_score

#To be implemented

class A_Star_Planner():
    __metaclass__ = ABCMeta    
    
    #Given start and goal configurations
    def __init__(self,goal,distanceWeight,angleWeight,distDisc,angleDisc,env,robot,room):
        self.robot = robot
        self.room = room
        self.start = list(self.robot.GetActiveDOFValues())
        self.goal = goal
        self.distanceWeight = distanceWeight
        self.angleWeight = angleWeight
        self.path = []
        self.env = env
        self.distDisc = distDisc
        self.angleDisc = angleDisc
        
        #Return world, a list of all possible configurations
        self.discretizeWorld()
        self.createDataStructs()
        super(A_Star_Planner,self).__init__()

    def roundList(self,l):
        #Lists are pass by reference I think. Technically don't need to return if
        #this is the case
        for i,j in enumerate(l):
            l[i] = int(round(j))
        return l

    #Convert angle to 0,2pi. CONTINUOUS ANGLE CASE
    def roundAngle(self,t):
        return t % (2 * np.pi)
        
    #Takes a box angle in and converts to valid range
    def roundBoxAngle(self,t):
        return t % (self.angleMaxlimit + 1)

    #Finds closest distance between 2 angles. CONTINUOUS ANGLE CASE
    def angleDistance(self,t1,t2):
        t11 = self.roundAngle(t1)
        t21 = self.roundAngle(t2)
        diff = abs(t21-t11)
        return min(diff,2 * np.pi - diff)

    #Finds closest distance between 2 angles. DISCRETE ANGLE CASE
    def angleBoxDistance(self,t1,t2):
        t11 = self.roundBoxAngle(t1)
        t21 = self.roundBoxAngle(t2)
        diff = abs(t21-t11)
        return min(diff,(self.angleMaxlimit + 1) - diff)

    #q in form [x,y,theta]. Discretized form
    def discrete2ContConfig(self,q):
        result = [q[0] * self.distDisc, q[1] * self.distDisc, q[2] * self.angleDisc]
        result[2] = self.roundAngle(result[2])
        return result

    def continous2DiscConfig(self,q):
        result = [q[0] / self.distDisc, q[1] / self.distDisc, q[2] / self.angleDisc]
        self.roundList(result)        
        result[2] = self.roundBoxAngle(result[2])
        return result

    def discretizeWorld(self):
        box = self.room.ComputeAABB()
        pos = box.pos()
        extents = box.extents()
    
        #Half dimensions of the x,y bounding box
        self.halfwidth = extents[0]
        self.halfheight = extents[1]
        
        #Get box coordinate limits
        self.boxXMaxlimit = math.ceil(self.halfwidth/self.distDisc)
        self.boxXMinlimit = -self.boxXMaxlimit
        
        self.boxYMaxlimit = math.ceil(self.halfheight/self.distDisc)
        self.boxYMinlimit = -self.boxYMaxlimit

        #Get the angle limits
        self.angleMinlimit = 0
        self.angleMaxlimit = int(math.ceil(2 * np.pi / self.angleDisc) - 1)
        
        #Discretize start and goal nodes
        self.boxGoal = self.continous2DiscConfig(self.goal)
        self.boxStart = self.continous2DiscConfig(self.start)
        
    #For data structures, I will work with the box coordinates only. I think
    #this is the best way.
    def createDataStructs(self):
        #Maps tuple version of box coords to corresponding g_score.
        #Won't store f_score since can be calculated. Use this to check
        #what's currently the best f_score for a node in case there's duplicates
        #in the priority queue
        self.gmap = {}

        #Maps tuple version of box coords to the parent (tuple) it came from
        #After visited
        self.visitmap = {}

        #Priority queue for A* algorithm
        self.pqueue = []

        #Set of colliding nodes
        #self.colliding = set([])

        #Maps (x,y) tuples to corresponding drawing handle. Also use this map
        #key list to prevent openrave from drawing nodes that have already been
        #drawn. This makes 3D viewer more efficient
        self.drawnqs = {}

    #Returns true if q is in collision, where q is a box coordinate
    def checkCollision(self,q):
        #Convert to continuous
        continous = self.discrete2ContConfig(q)

        with self.env:
            #Use openrave to check collision
            self.robot.SetActiveDOFValues(continous)        
            #TODO: Check collision with other bodies?      
            #Check collision with environment
            collresult = self.env.CheckCollision(self.robot)

        return collresult
    
    def checkInBounds(self,q):
        xl = self.boxXMinlimit <= q[0] and  q[0] <= self.boxXMaxlimit
        yl = self.boxYMinlimit <= q[1] and  q[1] <= self.boxYMaxlimit
        retl = (xl and yl)        
        return retl
    
    #Assumes using discrete coordinates
    def configDiff(self,q1,q2):
        return [abs(q1[0] - q2[0]), abs(q1[1] - q2[1]), self.angleBoxDistance(q1[2],q2[2])]

    #Use distanceWeight,angleWeight. Assumes discrete coordinates
    def manhattenDist(self,q1,q2):
        diff = self.configDiff(q1,q2)
        distc = self.distanceWeight * (diff[0] + diff[1])
        anglec = self.angleWeight * (1 - np.cos(self.angleDisc * diff[2]))
        return distc + anglec

    #Use distanceWeight,angleWeight. Assumes discrete coordinates
    def euclidDist(self,q1,q2):
        diff = self.configDiff(q1,q2)
        distc = self.distanceWeight * (np.sqrt(diff[0] * diff[0] + diff[1] * diff[1]))
        anglec = self.angleWeight * (1 - np.cos(self.angleDisc* diff[2]))
        return distc + anglec

    def get4Neighbors(self,q):
        neighbors = []
        #2 directions for each dimension
        for i in [-1,0,1]:
            for j in [-1,0,1]:
                for k in [-1,0,1]:
                    #Only consider neighbors where we have two are 0.
                    # 2n = 6
                    currcomb = [i,j,k]
                    if currcomb.count(0) == 2:
                        newc = np.array(q) + np.array([i,j,k])
                        newc = list(newc)
                        newc[2] = self.roundBoxAngle(newc[2])                    
                        neighbors.append(newc)
        return neighbors

    def get8Neighbors(self,q):
        neighbors = []
        #2 directions for each dimension
        for i in [-1,0,1]:
            for j in [-1,0,1]:
                for k in [-1,0,1]:
                    if i != 0 or j != 0 or k != 0:
                        newc = np.array(q) + np.array([i,j,k])
                        newc = list(newc)
                        newc[2] = self.roundBoxAngle(newc[2])                    
                        neighbors.append(newc)
        return neighbors

    #Draw colliding configuration. REQUIRES BOX COORDINATES
    #color:
    #0 = black. Actual path
    #1 = blue. Explored nodes
    #2 = red. Obstacles
    #TODO: Implement different colors
    def drawConfig(self,q,color):
        dq = (q[0],q[1])
        contq = self.discrete2ContConfig(q)
        
        #Draw black regardless since it's final path. Becareful since black
        #may draw over twice. Could harm performance
        if color == 0:
            self.drawnqs[dq] = None
            pcolors = np.array(((0,0,0,0.9)))
        #Blue. Blue can draw over red, but not other way around
        elif color == 1:
            if dq == (self.boxGoal[0],self.boxGoal[1]):
                return
            self.drawnqs[dq] = None
            pcolors = np.array(((0.145,0.913,0.819,0.9)))
        else:
            if dq not in self.drawnqs.keys(): 
                self.drawnqs[dq] = None
                
                #Red
                if color == 2:
                    pcolors = np.array(((0.913,0.145,0.152,0.9)))
            #Don't draw anything if already drawn
            else:
                return
        with self.env:
            self.drawnqs[dq] = self.env.plot3(points=np.array((contq[0],contq[1],0.1)),
                             pointsize=7.0,
                             colors=pcolors)
    
    def getPath(self):    
        return self.path
        
    #Runs Astar algorithm
    def searchPath(self):
        counter1 = 0
        counter2 = 0

        #Draw Start and Goal in black
        self.drawConfig(self.boxStart,0)
        self.drawConfig(self.boxGoal,0)

        try:
            #Start node has GScore of 0
            self.setGScore(self.boxStart,0)
            #Add start node to priority queue. No parent for it
            self.addNode2Queue(self.boxStart,None)
            
            #Repeatedly get top node from priority queue
            while not len(self.pqueue) == 0:
                nextnode = self.getNextValidPQNode()
                if nextnode is None:
                    return False
                
                current = nextnode.node
                parent = nextnode.parent
                #Mark as visited and add to visit list
                self.visitmap[current] = parent
    
                #Break if this is the goal node
                if current == tuple(self.boxGoal):
                    return True
                
                #Explore neighbors of current node
                neighbors = self.getNeighbors(current)
    
                #import pdb
                #pdb.set_trace()
                for neighbor in neighbors:
                    if neighbor[0] == self.boxGoal[0] and neighbor[1] == self.boxGoal[1]:
                        pass
                        #import pdb
                        #pdb.set_trace()
                    
                    #Ignore neighbors that are out of bounds
                    #See if neighbor has been visited                
                    if tuple(neighbor) not in self.visitmap.keys():               
                        #Check collision                
                        if (not self.checkCollision(neighbor)) and self.checkInBounds(tuple(neighbor)):
                            #Compute gscore and update gmap IF better than current GScore
                            neighbor_g = self.getGScore(current) + self.calcDistance(current,tuple(neighbor))                                                    
                            
                            if neighbor_g < self.getGScore(tuple(neighbor)):                            
                                self.setGScore(neighbor,neighbor_g)
                
                                #Add to priority queue with computed fscore
                                self.addNode2Queue(neighbor,current)
                                self.drawConfig(neighbor,1)
                        else:
                            #Draw colliding config
                            pass
                            self.drawConfig(neighbor,2)
                        counter2 += 1
                        print "Node not visited",counter2
                    else:
                        counter1 += 1
                        print "Node already visited", counter1
                        if counter1 == 3600:
                            pass
                        
            #Didn't find path
            #import pdb
            #pdb.set_trace()                        
            return False
        except openravepy.openrave_exception,e:
            print e            
            import pdb
            pdb.set_trace()
            #sys.traceback.print_tb(*sys.exc_info())
            print "exception occurred"

    #Given visitmap is filled with both the start and goal nodes, form the path
    def backtrackPath(self):
        currnode = tuple(self.boxGoal)
        while True:
            self.path.append(self.discrete2ContConfig(list(currnode)))
            #Draw it in black
            self.drawConfig(currnode,0)
            if self.visitmap[currnode] is None:
                break
            else:
                #Get the parent
                currnode = self.visitmap[currnode]
        #import pdb
        #pdb.set_trace()
        
        #Reverse the list in linear time.
        self.path.reverse()
        
        #Return precision to start and goal nodes
        self.path[0] = self.start
        self.path[len(self.path) - 1] = self.goal
        
    #Gets GScore of node. If not in gmap, then it's GScore is infinity    
    def getGScore(self,node):
        tnode = tuple(node)
        if tnode not in self.gmap.keys():
            self.gmap[tnode] = np.inf
        return self.gmap[tnode]        
   
    def setGScore(self,config,gscore):
        tconfig = tuple(config)
        self.gmap[tconfig] = gscore
        return
        
    #Gets top element from pq, throwing out old PQ nodes whose fscores are not up to date
    #Returns none if empty
    def getNextValidPQNode(self):
        while not len(self.pqueue) == 0:
            #Get top PQ Node
            nextnode = hq.heappop(self.pqueue)
            if nextnode.node == tuple(self.boxGoal):
                #import pdb
                #pdb.set_trace()
                pass

            confign = nextnode.node            
            
            #Check if fscore is up to date.
            truefscore = self.getGScore(confign) + self.calcDistance(confign,self.boxGoal)
            if truefscore ==  nextnode.f_score or (confign not in self.visitmap.keys()):
                return nextnode
            else:
                #Old fscore that's not up to date. there was a better fscore seen earlier.
                #Ignore this node
                print "Bad node in PQ"
                #import pdb
                #pdb.set_trace()
                pass
        return None
   
    #Adds box coordinate to priority queue, making appropriate priority queue entry
    def addNode2Queue(self,q,parent):
        tq = tuple(q)
        #Get g_score
        gscore = self.getGScore(tq)        
        #Compute f_score
        hscore = self.calcDistance(q,self.boxGoal)
        #node, parent, f_score
        entry = pqueue_entry(tq,parent,gscore + hscore)
        hq.heappush(self.pqueue,entry)
    
    #Distance is either L2 or L1. Depends on the subclass
    @abstractmethod
    def calcDistance(self,config1,config2):
        pass
    
    #Neighbors are obtained either for 4 or 8-connected space. Depends on the subclass
    @abstractmethod
    def getNeighbors(self,config):
        pass

class A_Star_Planner_4_manhatten(A_Star_Planner):
    def __init__(self,goal,distanceWeight,angleWeight,distDiscret,angleDiscret,env,robot,room):
        super(A_Star_Planner_4_manhatten, self).__init__(goal,distanceWeight,angleWeight,distDiscret,angleDiscret,env,robot,room)

    def calcDistance(self,config1,config2):
        return self.manhattenDist(config1,config2)
    def getNeighbors(self,config):
        return self.get4Neighbors(config)
        
class A_Star_Planner_8_manhatten(A_Star_Planner):
    def __init__(self,goal,distanceWeight,angleWeight,distDiscret,angleDiscret,env,robot,room):
        super(A_Star_Planner_8_manhatten, self).__init__(goal,distanceWeight,angleWeight,distDiscret,angleDiscret,env,robot,room)

    def calcDistance(self,config1,config2):
        return self.manhattenDist(config1,config2)
    def getNeighbors(self,config):
        return self.get8Neighbors(config)
        
class A_Star_Planner_4_euclidean(A_Star_Planner):
    def __init__(self,goal,distanceWeight,angleWeight,distDiscret,angleDiscret,env,robot,room):
        super(A_Star_Planner_4_euclidean, self).__init__(goal,distanceWeight,angleWeight,distDiscret,angleDiscret,env,robot,room)
    def calcDistance(self,config1,config2):
        return self.euclidDist(config1,config2)
    def getNeighbors(self,config):
        return self.get4Neighbors(config)
        
class A_Star_Planner_8_euclidean(A_Star_Planner):
    def __init__(self,goal,distanceWeight,angleWeight,distDiscret,angleDiscret,env,robot,room):
        super(A_Star_Planner_8_euclidean, self).__init__(goal,distanceWeight,angleWeight,distDiscret,angleDiscret,env,robot,room)
    def calcDistance(self,config1,config2):
        return self.euclidDist(config1,config2)
    def getNeighbors(self,config):
        return self.get8Neighbors(config)

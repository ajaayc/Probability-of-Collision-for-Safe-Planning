import gaussprop as gp
import numpy as np

prop = gp.Gauss_Prop(1,2,3)
prop.generateM_EKF([1,2,3])
prop.generateB([1,2,3],[1,2,3])
prop.generateA([1,2,3],[1,2,3])
prop.prediction([1,2,3],[1,2,3])
print prop.sensorReading(np.array([1,2,3]))
print prop.observation(np.array([8,2,3]),0)
prop.sampleOdometry([1,2,3],[1,2,3])
print prop.inverseOdometry([2,2,0],[3,3,0])

nominalcurrstate = np.array([1,1,np.pi/2])
estimatedcurrstate = np.array([1,1.41,np.pi/2])
nominalgoalstate = np.array([2,1,np.pi/2])
nominalcontrol = prop.inverseOdometry(nominalcurrstate,nominalgoalstate)

Lmatrix = prop.generateL(nominalcurrstate,estimatedcurrstate,nominalgoalstate,nominalcontrol)
print Lmatrix

print 'H matrix: ',prop.generateH([1,2,0])

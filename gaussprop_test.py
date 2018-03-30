import gaussprop as gp
import numpy as np

prop = gp.Gauss_Prop()
prop.generateM([1,2,3])
prop.generateV_EKF([1,2,3],[1,2,3])
prop.generateG_EKF([1,2,3],[1,2,3])
prop.prediction([1,2,3],[1,2,3])
print prop.sensorReading(np.array([1,2,3]))
print prop.observation(np.array([8,2,3]),0)

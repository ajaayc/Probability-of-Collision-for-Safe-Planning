import gaussprop as gp
import numpy as np

prop = gp.Gauss_Prop()
prop.generateM([1,2,3])
prop.generateV_EKF([1,2,3],[1,2,3])

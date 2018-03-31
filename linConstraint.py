import numpy as np

#A linear constraint
class linConstraint():
    def __init__(self,A,b,xmin,xmax,ymin,ymax):
        self.A = A
        self.b = b
        self.xmin = xmin
        self.xmax = xmax
        self.ymin = ymin
        self.ymax = ymax

    #state is the 3 x 1 state vector
    #Returns true if the constraint is passed
    def checkConstraint(self,state):
        ts = state[0:2]
        ts = np.array(ts)
        x = ts[0]
        y = ts[1]
        
        xcheck = (self.xmin <= x and x <= self.xmax)
        ycheck = (self.ymin <= y and y <= self.ymax)

        #Check line
        result = self.A * ts
        #See if <= b
        lcheck = result <= b
        lcheck = lcheck.all()

        return (lcheck and xcheck and ycheck)

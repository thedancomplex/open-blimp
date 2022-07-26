import numpy as np
from numpy.linalg import norm

class Wind:
    def __init__(self, centers, theta):
        self.centers = centers
        self.theta = theta
        
    def f(self, x):
        return np.sum(self.theta*self.rbf(x),axis=1,keepdims=True)
        
    def rbf(self, x):
        d = np.exp(-2*norm(x-self.centers,axis=0))
        return d
        
        
class windEstimate:
    def __init__(self, x0, centers, K=np.eye(2), gamma=0.1, theta0=None):
        theta = theta0
        if theta is None:
            theta = np.zeros((2,centers.shape[1]))
            
        self.W = Wind(centers, theta)
        self.x = x0.copy()
        self.z = x0.copy()
        self.e = self.x - self.z
        
        self.K = K
        self.gamma = gamma
        
    def predict(self, x):
        return self.W.f(x)
        
    def step(self, dx, u):
        beta = self.W.f(self.x) - self.W.f(self.z) + self.K @ self.e
    
        self.x += dx
        self.z += u + self.predict(self.z) + beta
        self.e = self.x - self.z

        self.W.theta += self.gamma*np.kron(self.e, self.W.rbf(self.x))
        

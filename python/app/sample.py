
import math
import numpy as np
import sklearn.preprocessing
from app.vector import Vector




class Sample:
    up = Vector(0,0,0)
    def __init__(self):
        self.imagePx = Vector(0,0)
        self.bufferIdx = 0
        self.gridIdx = Vector(0,0)
        self.p = Vector(0,0,0)  #point p that describes a plane with normal n
        self.n = Vector(0,0,0)
        self.angle = 0.0
        self.inGrid = True
        self.clusterId = -1

    def __gt__(self, other):
        return Sample.up*self.p > Sample.up * other.p
    def __lt__(self, other):
        return Sample.up * self.p < Sample.up * other.p
    def __eq__(self, other):
        return self.bufferIdx == other.bufferIdx
    def __iadd__(self,other):
        self.n += other.n
        self.p += other.p
        return self

    def __idiv__(self, other):
        if type(other) == float:
            self.n /= other
            self.n = self.n.normalize()
            self.p /= other
            return self
        else:
            pass

    def __truediv__(self, other):
        if type(other)  == float:
            self.n /= other
            self.n = self.n.normalize()
            self.p /= other
            return self
        else:
            pass



    def __rdiv__(self, other):
        if type(other)==float:
            self.n /= other
            self.n = self.n.normalize()
            self.p /= other
            return self
        else:
            pass

    def distance(self, other):
        d1 = math.fabs(self.n *(other.p-self.p))
        d2 = math.fabs(other.n*(other.p-self.p))
        #d3 = 1.0-np.dot(np.squeeze(np.asarray(self.n)), np.squeeze(np.asarray(other.n)))
        return d1+d2

    def plane_distance(self,vec):
        return self.n*(vec-self.p)

    #vec is vec2d here
    def evaluateAt(self,vec):
        return (self.p*self.n- vec[0]*self.n[0]-vec.y*self.n[2]) /self.n[3]

    #vec is vec3d
    def projectTo(self,vec):
        return (v.x,v.y, self.evaluateAt(vec))





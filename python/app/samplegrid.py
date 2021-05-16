from Vector3 import Vector3
from Vector2 import Vector2
from app.sample import Sample
import numpy as np
import sklearn.preprocessing
from app.vector import Vector
# ols
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from scipy.optimize import leastsq

import time
class SampleGrid:

    def __init__(self,config):
        self.samples = []
        self.prunedSamples = []
        self.planes = []
        self.planeAvg = []
        self.floorSegment = []
        self.floorPlane = Sample()
        self.planeCluster = []
        self.upVector = Vector(0,0,0)
        self.upVector[2] = 1
        self.config = config
        self.floorPlane.n = self.upVector


    def initialize(self):
        self.samples.clear()
        for k in range(0, self.config["samplesY"]):
            V = []
            for l in range(0, self.config["samplesX"]):
                i = l * (self.config["IMAGE_WIDTH"]-1) / (self.config["samplesX"]-1)
                j = self.config["IMAGE_HEIGHT"]-1-k * (self.config["IMAGE_HEIGHT"]-1) / (self.config["samplesY"]-1)
                sample = Sample()
                sample.gridIdx = Vector(l, k)
                sample.imagePx = Vector(i, j)
                sample.bufferIdx = int(i+j * self.config["IMAGE_WIDTH"])
                V.append(sample)

            self.samples.append(V)





    def update(self,points):
        block_sum = 0
        start = time.time()
        if self.upVector* self.floorPlane.n > 0.5:
            self.setUpVector(self.floorPlane.n)
        else:
            pass

        for i in range(0, len(self.samples)):
            for j in range(0, len(self.samples[i])):
                self.samples[i][j].p = Vector(*(points[self.samples[i][j].bufferIdx].tolist()))
                self.samples[i][j].inGrid = self.samples[i][j].p.any()

        #normal = np.zeros((3,1))
        end = time.time()
        print("Part 1 took: " + str((end - start) * 1000) + "ms")
        start = time.time()
        for i in range(0, len(self.samples)):
            for j in range(0, len(self.samples[i])):
                if not self.samples[i][j].inGrid:
                    continue
                upIdx = i + 1
                if i == len(self.samples) - 1 or not self.samples[upIdx][j].inGrid:
                    upIdx = i

                downIdx = i - 1
                if i == 0 or not self.samples[downIdx][j].inGrid:
                    downIdx = i

                rightIdx = j + 1
                if j == len(self.samples[i]) - 1 or not self.samples[i][rightIdx].inGrid:
                    rightIdx = j

                leftIdx = j - 1
                if j == 0 or not self.samples[i][leftIdx].inGrid:
                    leftIdx = j

                if upIdx == downIdx or leftIdx == rightIdx:
                    continue

                up = self.samples[upIdx][j]
                down = self.samples[downIdx][j]
                right = self.samples[i][rightIdx]
                left = self.samples[i][leftIdx]

                start = time.time()
                normal = ((up.p - down.p) ^ (right.p - left.p))
                normal = -normal
                normal = normal.normalize()
                self.samples[i][j].n = normal
                end =time.time()
                block_sum += (end-start)
        end = time.time()
        print("Critical block took: " + str(block_sum * 1000) + "ms")

    def setUpVector(self,up):
        self.upVector = up
        self.upVector = self.upVector.normalize()
        Sample.up = self.upVector


    def findFloor(self):
        start = time.time()
        self.prune()
        if len(self.prunedSamples) < 2:
            return self.floorPlane

        #sort by z coordinate of point
        self.prunedSamples.sort(key=lambda x: x.p[2])#*self.upVector,reverse=True)
        #self.prunedSamples.sort()

        self.planes.clear()
        self.planeAvg.clear()
        self.floorSegment.clear()
        self.floorPlane = self.prunedSamples[1]
        end = time.time()

        print("Pruning: " + str((end - start) * 1000) + "ms")
        start = time.time()
        for i in range(2, len(self.prunedSamples) - 1):
            if not self.isIn(self.prunedSamples[i].gridIdx):
                continue


            self.planeCluster.clear()
            self.floodFill(self.prunedSamples[i].gridIdx)
            if len(self.planeCluster) == 1:
                continue
            avg = Sample()
            avg.n[2] = 0
            for  j in range(0, len(self.planeCluster)):
                avg += self.planeCluster[j]

            avg /= float(len(self.planeCluster))

            self.planeAvg.append(avg)
            self.planes.append(self.planeCluster)

            if self.floorPlane.distance(avg) < self.config["mergeThreshold"]:
                self.floorPlane.p = (self.floorPlane.p * len(self.floorSegment) + avg.p * len(self.planeCluster)) / (len(self.floorSegment) + len(self.planeCluster))
                self.floorPlane.n = (self.floorPlane.n * len(self.floorSegment) + avg.n * len(self.planeCluster)) / (len(self.floorSegment) + len(self.planeCluster))
                self.floorPlane.n = self.floorPlane.n.normalize()
                self.floorSegment.extend(self.planeCluster)

            if len(self.floorSegment) * 20 < len(self.planeCluster):
                self.floorPlane = avg
                self.floorSegment.clear()
                self.floorSegment.extend(self.planeCluster)
        end = time.time()
        print("floodfill: " + str((end - start) * 1000) + "ms")
        if len(self.floorSegment) > 2:
            start = time.time()
            data_x = np.array([np.array([x.p[0], x.p[1],x.p[2]]) for x in self.floorSegment])
            data_y = np.array([x.p[2] for x in self.floorSegment])
            solution = np.linalg.lstsq(data_x,data_y, rcond=None)[0]
            self.floorPlane.p[2] = self.floorPlane.p[0] * solution[0] + self.floorPlane.p[1] * solution[1] + solution[2]
            self.floorPlane.n = Vector(-solution[0], -solution[1], 1).normalize()
            end = time.time()
            print("leastsq: " + str((end - start) * 1000) + "ms")

            # start = time.time()
            # data = np.array([x.p for x in self.floorSegment]).transpose()
            # p, n = self.planeFit(data)
            # end = time.time()
            # print("SVD: " + str((end - start) * 1000) + "ms")
            # self.floorPlane.p = Vector(*(p.tolist()))
            # self.floorPlane.n = Vector(*(n.tolist())).normalize()



        return self.floorPlane




    def floodFill(self, parentIdx):
        parent = self.samples[parentIdx[1]][parentIdx[0]]
        if not parent.inGrid:
            return

        parent.inGrid = False
        self.planeCluster.append(parent)
        if parent.gridIdx[0] > 0:
            childIdx = parent.gridIdx - Vector(1, 0)
            child = self.samples[childIdx[1]][childIdx[0]]

            if parent.distance(child) < self.config["floodThreshold"]:
                self.floodFill(childIdx)

        if parent.gridIdx[0] < self.config["samplesX"] - 1:
            childIdx = parent.gridIdx + Vector(1, 0)
            child = self.samples[childIdx[1]][childIdx[0]]
            if parent.distance(child) < self.config["floodThreshold"]:
                self.floodFill(childIdx)

        if parent.gridIdx[1] > 0:
            childIdx = parent.gridIdx - Vector(0, 1)
            child = self.samples[childIdx[1]][childIdx[0]]
            if parent.distance(child) < self.config["floodThreshold"]:
                self.floodFill(childIdx)

        if parent.gridIdx[1] < self.config["samplesY"] - 1:
            childIdx = parent.gridIdx + Vector(0, 1)
            child = self.samples[childIdx[1]][childIdx[0]]
            if parent.distance(child) < self.config["floodThreshold"]:
                self.floodFill(childIdx)



    def prune(self):
        self.prunedSamples.clear()
        for i in range(0, len(self.samples)):
            for j in range(0, len(self.samples[i])):
                if not self.samples[i][j].inGrid:
                    continue

                self.samples[i][j].angle = self.samples[i][j].n * self.upVector


                if self.samples[i][j].angle > self.config["pruneThreshold"]:
                    self.prunedSamples.append(self.samples[i][j])
                else:
                    self.samples[i][j].inGrid = False


    def isIn(self, gridIdx):
        return self.samples[gridIdx[1]][gridIdx[0]].inGrid

    def planeFit(self,points):
        """
        p, n = planeFit(points)

        Given an array, points, of shape (d,...)
        representing points in d-dimensional space,
        fit an d-dimensional plane to the points.
        Return a point, p, on the plane (the point-cloud centroid),
        and the normal, n.
        """
        points = np.reshape(points, (np.shape(points)[0], -1))  # Collapse trialing dimensions
        assert points.shape[0] <= points.shape[1], "There are only {} points in {} dimensions.".format(points.shape[1],  points.shape[0])
        ctr = points.mean(axis=1)
        x = points - ctr[:, np.newaxis]
        M = np.dot(x, x.T)  # Could also use np.cov(x) here.
        return ctr, np.linalg.svd(M)[0][:, -1]

    def drawSamples(self):
        pass


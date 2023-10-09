import numpy as np
import math
# import random
class gpsNoise(object):
    def __init__(self):
        self.f = 10.0
        self.wd = 1.0
        self.wv = 0.08
        self.wa = 0.00875
        self.s = 0.016289174978068626
        self.mv = self.wv/2
        self.ma = self.wa/2
        self.widthOfDistr = self.wd/4
        self.crntValueOfRndmWalk = 0.0
        self.dCrntValue = 0.0
        self.timeOfNextUpdate = 0.0
        self.sigma = self.s
        self.maxv = self.wd*self.mv/8
        self.maxa = self.wd*self.ma/8
        self.maxNudge = 0.15*self.sigma

    def generateNoise(self):
        for i in range(0,5):
            self.randomWalk()
        return self.crntValueOfRndmWalk
    
    def randomWalk(self):
        mean = self.maxNudge*self.nudge()
        a = np.random.normal(mean, self.sigma)
        if(a>self.maxa):
            a = self.maxa
        elif(-a>self.maxa):
            a = -self.maxa
        self.crntValueOfRndmWalk = self.crntValueOfRndmWalk+self.dCrntValue+a
        self.dCrntValue = self.dCrntValue+a
        if (self.dCrntValue>self.maxv):
            self.dCrntValue = self.maxv
        elif (-self.dCrntValue>self.maxv):
            self.dCrntValue = -self.maxv

    def nudge(self):
        myNudge = -self.crntValueOfRndmWalk/self.widthOfDistr
        if(myNudge>1):
            myNudge = 1
        elif(myNudge<-1):
            myNudge = -1
        return myNudge


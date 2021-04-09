#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr  6 15:51:58 2021

@author: erik
"""

import numpy as np

class droneSTAB():
    
    def __init__(self):
        self.angComp = 1
        self.thrustComp = 1
        self._zref = 1
        
    def yawStab(self, yaw, yawRef):
        Kp = 1
        maxRate = 60
        return min(maxRate,max(-maxRate,Kp*(yaw-yawRef)))
    
    def rollPitchStabPos(self, pos,vel,yaw,setPoint):
        kp = 35
        kv = 60
        maxAng = 15
        alpha = -yaw*np.pi/180
        tranX = np.cos(alpha)*setPoint[0]-np.sin(alpha)*setPoint[1]
        tranY = np.cos(alpha)*setPoint[1]+np.sin(alpha)*setPoint[0]
        pitch = min(maxAng,max(-maxAng,((tranX-pos[0])*kp-vel[0]*kv)*self.thrustComp))
        roll = min(maxAng,max(-maxAng,((tranY-pos[1])*kp-vel[1]*kv)*self.thrustComp))
        #self.angComp = np.sqrt(1+(np.pi*pitch/180)**2+(np.pi*roll/180)**2)
        return np.array([-roll,pitch])
    
    def thrustStab(self, z, zvel, zref):
        
        deltaH = 0.5
        if zref-z > deltaH:
            self._zref = z+deltaH
            #print('Step up')
        elif zref-z < -deltaH:
            self._zref = z-deltaH
            #print('Step down!')
           
        else:
            self._zref = zref
        
        
        #self._zref = zref
        minThrust = 25000
        maxThrust = 60000
        zeroG = 36000
        e = self._zref-z
        Kp = 35000
        Kv = 50000
        thrust = min(maxThrust,max(minThrust,int(self.angComp*(Kp*e-Kv*zvel + zeroG))))
        #self.thrustComp = zeroG/thrust
        return thrust
        
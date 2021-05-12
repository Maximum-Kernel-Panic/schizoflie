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
        alpha = 0.4*yaw*np.pi/180
        #print(alpha)
        #print(round(alpha,2))
        x_sp = setPoint[0]
        y_sp = setPoint[1]
        #tranX = np.cos(alpha)*setPoint[0]-np.sin(alpha)*setPoint[1]
        #tranY = np.cos(alpha)*setPoint[1]+np.sin(alpha)*setPoint[0]
        #pitch = min(maxAng,max(-maxAng,((tranX-pos[0])*kp-vel[0]*kv)*self.thrustComp))
        #roll = min(maxAng,max(-maxAng,((tranY-pos[1])*kp-vel[1]*kv)*self.thrustComp))
        
        u_x = ((x_sp-pos[0])*kp-vel[0]*kv)*self.thrustComp
        u_y = ((y_sp-pos[1])*kp-vel[1]*kv)*self.thrustComp
        
        #pitch = u_x
        #roll = u_y
        
        nsin = np.sin(alpha)
        ncos = np.cos(alpha)
        
        pitch = ncos*u_x + nsin*u_y
        roll = ncos*u_y - nsin*u_x
        #print("Pitch u: {}, Roll u: {}".format(round(pitch,0),round(roll,0)))
        #print("Pitch: {}% pitch {}% roll,  Roll: {}% pitch {}% roll".format((round(ncos),round(nsin),round(ncos)))
        
        #pitch = np.cos(alpha)*u_x - np.sin(alpha)*
        
        fac = maxAng/np.sqrt((pitch**2 + roll**2 + 1e-5))
        if fac < 1:
            pitch = pitch*fac
            roll = roll*fac
        
            
        #self.angComp = np.sqrt(1+(np.pi*pitch/180)**2+(np.pi*roll/180)**2)
        return np.array([-roll,pitch])

    def thrustStab(self, z, zvel, zref, comp=False, pitch=0, roll=0):

        deltaH = 0.5
        if zref-z > deltaH:
            self._zref = z+deltaH
            #print('Step up')
        elif zref-z < -deltaH:
            self._zref = z-deltaH
            #print('Step down!')

        else:
            self._zref = zref
            
        if comp:
            pitch = min(abs(pitch*np.pi/180),0.1745)
            roll = min(abs(roll*np.pi/180),0.1745)
            angSqr = pitch**2 + roll**2
            angComp = 1/(1-0.5*angSqr)

        #self._zref = zref
        minThrust = 25000
        maxThrust = 65535
        zeroG = 38500
        e = self._zref-z
        Kp = 35000
        Kv = 50000
        thrust = min(maxThrust,max(minThrust,int(angComp*(Kp*e-Kv*zvel + zeroG))))
        #self.thrustComp = zeroG/thrust
        return thrust

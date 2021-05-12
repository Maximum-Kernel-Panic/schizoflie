#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 11 13:57:34 2021

@author: erik
"""
import threading
import time
import sys
import logging
import tkinter
import numpy as np
from threading import Thread

class PositionLogger(threading.Thread):
    
    def __init__(self, controller):
        threading.Thread.__init__(self)
        self.controller = controller
        self.period_in_s = 0.05
        self.rate = 20
        self.logging_wait = 12
        self.logging_time = 15
        self.start_time = time.time()
        self.data_pts = self.logging_time*self.rate
        self.data = np.zeros((7,self.data_pts))
                
        
        
    def run(self):
        
        while time.time()-self.start_time < self.logging_wait:
            time_start = time.time()
            self.loop_sleep(time_start)
            
            
        time_logging_start = time.time()
        
        for k in range(self.data_pts):
            time_start = time.time()
            self.data[0,k] = self.controller.x
            self.data[1,k] = self.controller.y
            self.data[2,k] = self.controller.z
            self.data[3,k] = time_start - time_logging_start
            self.data[4,k] = self.controller.setPointX
            self.data[5,k] = self.controller.setPointY
            self.data[6,k] = self.controller.setPointZ
            self.loop_sleep(time_start)
            
            
            
        np.savetxt("joyck_circle", self.data)
    
    
    def loop_sleep(self, timeStart):
        """ Sleeps the control loop to make it run at a specified rate """
        deltaTime = 1.0/float(self.rate) - (time.time() - timeStart)
        if deltaTime > 0:
            time.sleep(deltaTime)
        else:
            print('Could not make controller loop deadline')
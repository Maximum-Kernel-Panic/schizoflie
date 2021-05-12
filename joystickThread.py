#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 25 17:43:10 2021

@author: erik
"""

import threading
import time
import sys
import logging
import tkinter
import random
import numpy as np
import pygame
from threading import Thread


class joyStickThread(threading.Thread):

    
    def __init__(self):
        threading.Thread.__init__(self)
        
        self.daemon = True
        self.turnOff = False
        self.wait_in_s = 0.05
        self.inactivity_timer = 0
        self.tol = 0.04
        self.parking_timer = 1
        self.gain = 20;
        self.yaw_gain = 5 * self.gain
        self.hat = None
        
    def run(self):
        
        pygame.init()
        pygame.joystick.init()
        isOn = pygame.joystick.get_init()
        sticky = pygame.joystick.Joystick(0)
        sticky.init()
        pygame.event.get()
        self.pitch_origin = sticky.get_axis(1)
        self.roll_origin = sticky.get_axis(0)
        self.yaw_origin = sticky.get_axis(2)
        
        #self.pitch_origin = 0
        #self.roll_origin = 0
        #self.yaw_origin = 0
        
        
        while not self.turnOff:
            sticky = pygame.joystick.Joystick(0)
            sticky.init()
            pygame.event.get()
            pitchdata = sticky.get_axis(1) - self.pitch_origin
            rolldata = sticky.get_axis(0) - self.roll_origin
            yawdata = sticky.get_axis(2) - self.yaw_origin
            hasHat = sticky.get_numhats() >= 1
            if hasHat:
                hatData = sticky.get_hat(0)
            
                
                
            if not hasHat:
                if (abs(pitchdata) < self.tol) and (abs(rolldata) < self.tol) and (abs(yawdata)<self.tol):
                    self.inactivity_timer = self.inactivity_timer + self.wait_in_s
                    
            if hasHat:
                if (abs(pitchdata) < self.tol) and (abs(rolldata) < self.tol) and (abs(yawdata)<self.tol) and (hatData==0):
                    self.inactivity_timer = self.inactivity_timer + self.wait_in_s
            else:
                self.inactivity_timer = 0
                
            
                
                
            if self.inactivity_timer > self.parking_timer:
                self.pitch = 0
                self.roll = 0
                self.yaw = 0
                self.hat = 0
                    
            else:
                self.pitch = self.gain*(pitchdata)
                self.roll = self.gain*(rolldata)
                self.thrust = sticky.get_axis(3) 
                self.yaw = self.yaw_gain*(yawdata)
                if hasHat:
                    self.hat = hatData[1]
            
            time.sleep(self.wait_in_s)
            
        
        
        
        '''
if __name__ == "__main__":

    stick  = joyStickThread()
    stick.start()
'''
        
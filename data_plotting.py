#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 11 14:17:38 2021

@author: erik
"""
import matplotlib.pyplot as plt
import time
import sys
import logging
import tkinter
import numpy as np

data = np.loadtxt("joystick_circle")
#data = data[:,10:140]
print(data)


'''
plt.subplot(3,1,1)
plt.plot(data[3],data[0]-0.5)
plt.plot(data[3],data[4]-0.5)
plt.grid()
plt.ylabel("X (m)")
#plt.title("Position X")
plt.ylim([-0.2,0.2])
#plt.ylim([-0.1,1.7])

plt.subplot(3,1,2)
plt.plot(data[3], data[1]-0.5)
plt.plot(data[3], data[5]-0.5)
plt.ylim([-0.2,0.2])
#plt.ylim([-0.1,1.7])
plt.grid()
plt.ylabel("Y (m)")
#plt.title("Position Y")

plt.subplot(3,1,3)
zpos = plt.plot(data[3], data[2])
zref = plt.plot(data[3], data[6])
plt.ylim([0.3,2])
#plt.title("Position Z")
plt.xlabel("Time (s)")
plt.grid()
plt.ylabel("Z (m)")
plt.legend(["Position", "Set point"],loc="lower right")

plt.show()
'''

t = np.linspace(0,2*np.pi,500)
y = 0.5*np.sin(t)
x = 0.5*np.cos(t)

plt.figure(0)
plt.plot(data[1]-0.5,data[0])
plt.plot(x,y)
plt.ylim([-0.7,0.7])
plt.xlim([-0.7,0.7])
plt.legend(["Path", "Target"], loc="center")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.grid()
plt.show()

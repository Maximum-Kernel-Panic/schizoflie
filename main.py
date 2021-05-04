import threading
import time
import sys
import logging
import numpy as np
#import keyboard
from threading import Thread
import joystickThread as jst
import transformations as trans
#from pyquaternion import Quaternion
from droneSTAB import droneSTAB
from controller_multi import Controller
from GUI import SimpleWindu
import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig



logging.basicConfig(level=logging.ERROR)

URI = 'radio://0/80/2M'
def __init__():
    print("leggo")

if __name__ == "__main__":
    stick=jst.joyStickThread()
    stick.start()
    cflib.crtp.init_drivers(enable_debug_driver=False)
    control = Controller(URI)
    control.set_joystick(stick)
    gui=SimpleWindu(control)
    gui.start()
    #gui2=GUITest()
    #gui2.start()

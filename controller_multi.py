import threading
import time
import sys
import logging
import tkinter
import numpy as np
from threading import Thread
import joystickThread as jst
import transformations as trans
#from pyquaternion import Quaternion
from droneSTAB import droneSTAB

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig


logging.basicConfig(level=logging.ERROR)

URI = 'radio://0/80/2M'

class Controller:
    def __init__(self,link_uri):
        self.joystick = None
        self.rate = 100 # [Hz]
        self.cf = Crazyflie(rw_cache='./cache')
        self.stabilizer=droneSTAB()
        self.joystickMode = False
        self.KILL = False
        self.KILL_HEIGHT = 0.1
        self.flying = False
        self.hasParked = False
        self.hasLanded = False
        self.x=0
        self.y=0
        self.z=0

        self.smoothing_number = 2
        #sets the velocity estimation weights based on smoothing number
        if self.smoothing_number == 2:
            self.vel_weights = np.array([0.5, 0.5])
        elif self.smoothing_number == 3:
            self.vel_weights = np.array([0.1, 0.3, 0.6])
        elif self.smoothing_number  == 4:
            self.vel_weights = np.array([0.05, 0.2, 0.25, 0.4])
        else:
            self.vel_weights = np.ones(self.smoothing_number)* 1/self.smoothing_number

        self.vx_reg = np.zeros(self.smoothing_number)
        self.vy_reg = np.zeros(self.smoothing_number)
        self.vz_reg = np.zeros(self.smoothing_number)
        self.vx_avg = 0
        self.vy_avg = 0
        self.vz_avg = 0

        # add a bunch of callbacks to initialize logging or
        # print out connection problems
        self.cf.connected.add_callback(self._connected)
        self.cf.disconnected.add_callback(self._disconnected)
        self.cf.connection_failed.add_callback(self._connection_failed)
        self.cf.connection_lost.add_callback(self._connection_lost)

        # Control period. [ms]
        self.period_in_ms = 10  # a random number
        self.joystickHatDelta = 0.4*self.period_in_ms/1000
        # Pose estimate from the Kalman filter
        self.pos = np.r_[0.0, 0.0, 0.0]
        self.vel = np.r_[0.0, 0.0, 0.0]
        self.attq = np.r_[0.0, 0.0, 0.0, 1.0]
        self.R = np.eye(3)

        # Attitide (roll, pitch, yaw) from stabilizer
        self.stab_att = np.r_[0.0, 0.0, 0.0]

        # This makes Python exit when this is the only thread alive.
        self.daemon = True

        # Start connection with drone!
        print('Trying to connect to %s' % link_uri)
        self.cf.open_link(link_uri)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        log_stab_att = LogConfig(name='Stabilizer', period_in_ms=self.period_in_ms)
        log_stab_att.add_variable('stabilizer.roll', 'float')
        log_stab_att.add_variable('stabilizer.pitch', 'float')
        log_stab_att.add_variable('stabilizer.yaw', 'float')
        self.cf.log.add_config(log_stab_att)

        log_pos = LogConfig(name='Kalman Position', period_in_ms=self.period_in_ms)
        log_pos.add_variable('kalman.stateX', 'float')
        log_pos.add_variable('kalman.stateY', 'float')
        log_pos.add_variable('kalman.stateZ', 'float')
        self.cf.log.add_config(log_pos)

        log_vel = LogConfig(name='Kalman Velocity', period_in_ms=self.period_in_ms)
        log_vel.add_variable('kalman.statePX', 'float')
        log_vel.add_variable('kalman.statePY', 'float')
        log_vel.add_variable('kalman.statePZ', 'float')
        self.cf.log.add_config(log_vel)

        log_att = LogConfig(name='Kalman Attitude',
                            period_in_ms=self.period_in_ms)
        log_att.add_variable('kalman.q0', 'float')
        log_att.add_variable('kalman.q1', 'float')
        log_att.add_variable('kalman.q2', 'float')
        log_att.add_variable('kalman.q3', 'float')
        self.cf.log.add_config(log_att)

        if log_stab_att.valid and log_pos.valid and log_vel.valid and log_att.valid:
            log_stab_att.data_received_cb.add_callback(self._log_data_stab_att)
            log_stab_att.error_cb.add_callback(self._log_error)
            log_stab_att.start()

            log_pos.data_received_cb.add_callback(self._log_data_pos)
            log_pos.error_cb.add_callback(self._log_error)
            log_pos.start()

            log_vel.error_cb.add_callback(self._log_error)
            log_vel.data_received_cb.add_callback(self._log_data_vel)
            log_vel.start()

            log_att.error_cb.add_callback(self._log_error)
            log_att.data_received_cb.add_callback(self._log_data_att)
            log_att.start()
        else:
            raise RuntimeError('One or more of the variables in the configuration was not'
                               'found in log TOC. Will not get any position data.')
        Thread(target=self.run).start()

    def _disconnected(self, link_uri):
        print('Disconnected from %s' % link_uri)

    def _connection_failed(self, link_uri, msg):
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _log_data_stab_att(self, timestamp, data, logconf):
        self.stab_att = np.r_[data['stabilizer.roll'],
                              data['stabilizer.pitch'],
                              data['stabilizer.yaw']]
        self.pitch = data['stabilizer.pitch']
        self.roll = data['stabilizer.roll']
        self.yaw = data['stabilizer.yaw']

    def _log_data_pos(self, timestamp, data, logconf):
        self.pos = np.r_[data['kalman.stateX'],
                         data['kalman.stateY'],
                         data['kalman.stateZ']]
        self.x = data['kalman.stateX']
        self.y = data['kalman.stateY']
        self.z = data['kalman.stateZ']

    def _log_data_vel(self, timestamp, data, logconf):
        vel_bf = np.r_[data['kalman.statePX'],
                       data['kalman.statePY'],
                       data['kalman.statePZ']]
        self.vx = data['kalman.statePX']
        self.vy = data['kalman.statePY']
        self.vz = data['kalman.statePZ']
        self.vel = np.dot(self.R, vel_bf)
        self.update_speed_regs()

    def _log_data_att(self, timestamp, data, logconf):
        # NOTE q0 is real part of Kalman state's quaternion, but
        # transformations.py wants it as last dimension.
        self.attq = np.r_[data['kalman.q1'], data['kalman.q2'],
                          data['kalman.q3'], data['kalman.q0']]
        # Extract 3x3 rotation matrix from 4x4 transformation matrix
        self.R = trans.quaternion_matrix(self.attq)[:3, :3]        #Blåe's line
        #self.R = Quaternion(self.attq)                              #Joar's line
        #r, p, y = trans.euler_from_quaternion(self.attq)

    def _log_error(self, logconf, msg):
        print('Error when logging %s: %s' % (logconf.name, msg))

    def make_position_sanity_check(self):
        # We assume that the position from the LPS should be
        # [-20m, +20m] in xy and [0m, 5m] in z
        if np.max(np.abs(self.pos[:2])) > 20 or self.pos[2] < 0 or self.pos[2] > 5:
            raise RuntimeError('Position estimate out of bounds', self.pos)

    def reset_estimator(self):
        self.cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self.cf.param.set_value('kalman.resetEstimation', '0')
        # Sleep a bit, hoping that the estimator will have converged
        # Should be replaced by something that actually checks...
        time.sleep(1.5)

    def check_kill_conditions(self):
        if (self.pitch**2 + self.roll**2) > 45**2:
            self.KILL = True
        


    def update_speed_regs(self):
        for i in range(self.smoothing_number - 1):
            self.vx_reg[i] = self.vx_reg[i]
            self.vy_reg[i] = self.vy_reg[i]
            self.vz_reg[i] = self.vz_reg[i]
        self.vx_reg[-1] = self.vx
        self.vy_reg[-1] = self.vy
        self.vz_reg[-1] = self.vz

        self.vx_avg = np.dot(self.vx_reg, self.vel_weights)
        self.vy_avg = np.dot(self.vy_reg, self.vel_weights)
        self.vz_avg = np.dot(self.vz_reg, self.vel_weights)

    def enable_joystick_mode(self):
        if self.joystick is None:
            print("Trying to enable manual mode with no joystick connected!")
        else:
            self.joystickMode = True

    def disable_joystick_mode(self):
        self.joystickMode = False

    def set_joystick(self, joystick):
        #if joystick is jst.joyStickThread:
        #    self.joystick = joystick
        #else:
        #    print("Fatal error: Incorrect reference")
        self.joystick = joystick

    def setRef(self,xref,yref,zref):
        self.setPointX=xref
        self.setPointY=yref
        self.setPointZ=zref

    def takeoff(self):
        self.setPointX = self.x
        self.setPointY = self.y
        self.setPointZ = 1
        self.flying=True

    def run(self):
        """Control loop"""
        try:
            print('Waiting for position estimate to be good enough...')
            self.reset_estimator()
            self.make_position_sanity_check();
            self.originX = self.x
            self.originY = self.y
            self.setPointX = self.originX
            self.setPointY = self.originY
            self.setPointZ = 1


            startTime = time.time()

            while not self.KILL:
                timeStart = time.time()

                while not self.flying:
                    if not self.hasLanded:
                        self.setPointX = self.x
                        self.setPointY = self.y
                        landTime = 2
                        landStartTime = time.time()
                        while (time.time() - landStartTime) < landTime and not self.KILL and self.z > self.KILL_HEIGHT:
                            timeStart = time.time()
                            attitude=self.stabilizer.rollPitchStabPos(np.array([self.x,self.y]),np.array([self.vx_avg,self.vy_avg]),self.yaw,np.array([self.setPointX,self.setPointY]))
                            thrust=self.stabilizer.thrustStab(self.z, self.vz_avg,self.KILL_HEIGHT+0.1, True, self.roll, self.pitch)
                            yaw=self.stabilizer.yawStab(self.yaw,0)
                            self.cf.commander.send_setpoint(attitude[0],attitude[1],0,thrust)
                            self.loop_sleep(timeStart)

                    self.hasLanded = True
                    timeStart = time.time()
                    self.cf.commander.send_setpoint(0,0,0,0)
                    self.loop_sleep(timeStart)

                self.hasLanded = False
                self.check_kill_conditions()
                self.update_speed_regs()


                if self.joystickMode:
                    pitch = self.joystick.pitch
                    roll = self.joystick.roll
                    yaw = self.joystick.yaw
                    hat = self.joystick.hat
                    if hat is not None:
                        self.setPointZ = self.setPointZ + hat*self.joystickHatDelta
                        
                    thrust=self.stabilizer.thrustStab(self.z, self.vz_avg, self.setPointZ ,self.joystickMode)

                    if (pitch == 0) and (roll == 0) and (yaw == 0):
                        if not self.hasParked:
                            self.setPointX = self.x
                            self.setPointY = self.y

                        self.hasParked = True
                        attitude=self.stabilizer.rollPitchStabPos(np.array([self.x,self.y]),np.array([self.vx_avg,self.vy_avg]),self.yaw,np.array([self.setPointX,self.setPointY]))
                        thrust=self.stabilizer.thrustStab(self.z, self.vz_avg, self.setPointZ, True, self.roll, self.pitch)
                        yaw=self.stabilizer.yawStab(self.yaw,0)
                        self.cf.commander.send_setpoint(attitude[0],attitude[1],yaw,thrust)

                    else:
                        self.hasParked = False
                        self.cf.commander.send_setpoint(roll, -pitch ,yaw,thrust)

                else:
                    attitude=self.stabilizer.rollPitchStabPos(np.array([self.x,self.y]),np.array([self.vx_avg,self.vy_avg]),self.yaw,np.array([self.setPointX,self.setPointY]))
                    thrust=self.stabilizer.thrustStab(self.z, self.vz_avg, self.setPointZ, True, self.roll, self.pitch)
                    yaw=self.stabilizer.yawStab(self.yaw,0)
                    
                    self.cf.commander.send_setpoint(attitude[0],attitude[1],yaw,thrust)
                    



                self.loop_sleep(timeStart)

            if self.KILL:
                print('KILL detected')
            self.cf.commander.send_setpoint(0,0,0,0)
            self.cf.close_link()

        except (KeyboardInterrupt):
            print("KEYBOARD INTERRUPT")
            for i in range(130):
                attitude=self.stabilizer.rollPitchStabPos(np.array([self.x,self.y]),np.array([self.vx,self.vy]),self.yaw,np.array([self.originX,self.originY]))
                thrust=self.stabilizer.thrustStab(self.z, self.vz,0.05, True, self.roll, self.pitch)
                yaw=self.stabilizer.yawStab(self.yaw,0)
                self.cf.commander.send_setpoint(attitude[0],attitude[1],0,thrust)
                time.sleep(0.01)
            self.cf.close_link()

    def loop_sleep(self, timeStart):
        """ Sleeps the control loop to make it run at a specified rate """
        deltaTime = 1.0/float(self.rate) - (time.time() - timeStart)
        if deltaTime > 0:
            time.sleep(deltaTime)
        else:
            print('Could not make controller loop deadline')

if __name__ == "__main__":
    stick = jst.joyStickThread()
    stick.start()
    print("initiated joystick")
    cflib.crtp.init_drivers(enable_debug_driver=False)
    control = Controller(URI)
    control.set_joystick(stick)

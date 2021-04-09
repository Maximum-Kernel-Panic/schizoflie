import threading
import time
import sys
import logging
import tkinter
import numpy as np
from threading import Thread
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
        self.rate = 100 # [Hz]
        self.cf = Crazyflie(rw_cache='./cache')
        self.stabilizer=droneSTAB()
        self.KILL = False
        self.KILL_HEIGHT = 0.1
        self.smoothing_number = 2
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

    def _log_data_att(self, timestamp, data, logconf):
        # NOTE q0 is real part of Kalman state's quaternion, but
        # transformations.py wants it as last dimension.
        self.attq = np.r_[data['kalman.q1'], data['kalman.q2'],
                          data['kalman.q3'], data['kalman.q0']]
        # Extract 3x3 rotation matrix from 4x4 transformation matrix
        self.R = trans.quaternion_matrix(self.attq)[:3, :3]
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
        if abs(self.pitch) > 35:
            self.KILL = True
        if abs(self.roll) > 35:
            self.KILL = True
            
    def update_speed_regs(self):
        for i in range(self.smoothing_number - 1):
            self.vx_reg[i] = self.vx_reg[i]
            self.vy_reg[i] = self.vy_reg[i]
            self.vz_reg[i] = self.vz_reg[i]
        self.vx_reg[-1] = self.vx
        self.vy_reg[-1] = self.vy
        self.vz_reg[-1] = self.vz
        self.vx_avg = np.sum(self.vx_reg)/self.smoothing_number
        self.vy_avg = np.sum(self.vy_reg)/self.smoothing_number
        self.vz_avg = np.sum(self.vz_reg)/self.smoothing_number
            
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
            
            setPointListX = self.originX + np.array([0,0,0,0,0,0,0])
            setPointListY = self.originY + np.array([0,1.5,0,1.5,2,0,0])
            setPointListZ = np.array([1, 1, 1, 1, 1, 1,1])
            setPointTime = np.array([3, 7, 11, 15, 19, 23, 27])

            self.cf.commander.send_setpoint(0, 0, 0, 0)
            timereg = 0
            flyTime = setPointTime[-1]
            startTime = time.time()
            
            while (time.time() - startTime) < flyTime and not self.KILL:
                timeStart = time.time()
                self.check_kill_conditions()
                self.update_speed_regs()
                
                if timeStart - startTime > setPointTime[timereg]:
                    self.setPointX = setPointListX[timereg]
                    self.setPointY = setPointListY[timereg]
                    self.setPointZ = setPointListZ[timereg]
                    timereg = timereg + 1
                    
                #attitude=self.stabilizer.rollPitchStabPos(np.array([self.x,self.y]),np.array([self.vx,self.vy]),self.yaw,np.array([self.setPointX,self.setPointY]))
                #thrust=self.stabilizer.thrustStab(self.z, self.vz, self.setPointZ)

                attitude=self.stabilizer.rollPitchStabPos(np.array([self.x,self.y]),np.array([self.vx_avg,self.vy_avg]),self.yaw,np.array([self.setPointX,self.setPointY]))
                thrust=self.stabilizer.thrustStab(self.z, self.vz_avg, self.setPointZ)
                yaw=self.stabilizer.yawStab(self.yaw,0)
                #print('pitch: {} roll: {}'.format(round(self.pitch,2),round(self.roll,2)))
                print('x:{} y:{} z:{} , setpoint({},{},{}), origin: ({},{})'.format(round(self.x,2),round(self.y,2),round(self.z,2),round(self.setPointX,2),round(self.setPointY,2),round(self.setPointZ,2),round(self.originX,2),round(self.originY,2)))
                #print('vx_avg:{} vy_avg:{} vz_avg:{} origin: ({},{})'.format(round(self.vx_avg,2),round(self.vy_avg,2),round(self.vz_avg,2),round(self.originX,2),round(self.originY,2)))
                #print('Flytime: {}'.format(time.time() - startTime))
                self.cf.commander.send_setpoint(attitude[0],attitude[1],yaw,thrust)
                self.loop_sleep(timeStart)
                
                
            self.setPointX = self.originX
            self.setPointY = self.originY
            landTime = 2
            startTime = time.time()
            while (time.time() - startTime) < landTime and not self.KILL and self.z > self.KILL_HEIGHT: 
                timeStart = time.time()
                self.update_speed_regs()
                attitude=self.stabilizer.rollPitchStabPos(np.array([self.x,self.y]),np.array([self.vx_avg,self.vy_avg]),self.yaw,np.array([self.setPointX,self.setPointY]))
                thrust=self.stabilizer.thrustStab(self.z, self.vz_avg,self.KILL_HEIGHT+0.1)                
                yaw=self.stabilizer.yawStab(self.yaw,0)
                self.cf.commander.send_setpoint(attitude[0],attitude[1],0,thrust)
                #print('Landtime: {}'.format(time.time() - startTime))
                self.loop_sleep(timeStart)
            self.cf.commander.send_setpoint(0,0,0,0)
            self.cf.close_link()
            
        except (KeyboardInterrupt):
            print("Hi")
            for i in range(130):
                attitude=self.stabilizer.rollPitchStabPos(np.array([self.x,self.y]),np.array([self.vx,self.vy]),self.yaw,np.array([self.originX,self.originY]))
                thrust=self.stabilizer.thrustStab(self.z, self.vz,0.05)
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
            
            
            
class windowThread(threading.Thread):
    
    
    def __init__(self):
        threading.Thread.__init__(self)
        self.daemon = True
        self.turnOff = False
        
    def fly(self):
        print('YEET')
        raise Exception()
    
    def killSwitch(self):
        self.turnOff = True

    def run(self):
        window = tkinter.Tk()
        button = tkinter.Button(
                text="Allah hu ackbar",
                command = self.fly,
                width=25,
                height=5,
                bg="black",
                fg="red",
                )
        button.pack()
        window.mainloop()

if __name__ == "__main__":
    gui = windowThread()
    gui.start()
    cflib.crtp.init_drivers(enable_debug_driver=False)
    control = Controller(URI)
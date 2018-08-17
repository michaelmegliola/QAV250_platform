import numpy as np
import time
import Adafruit_GPIO.I2C as I2C
from Adafruit_PCA9685 import *
from Adafruit_BNO055 import BNO055
from nxp_imu import IMU
import VL53L1X
import threading
from ahrs import *

FRONT_RIGHT = 0
REAR_RIGHT = 1
REAR_LEFT = 2
FRONT_LEFT = 3

FORE = 0
STARBOARD = 1
AFT = 2
PORT = 3

PWM_FREQ_HZ = 60                                     # cycles per second, do not exceed 240!

ESC_PWM_MIN = int((0.001 / (1/PWM_FREQ_HZ)) * 4096)  # esc off = pulse width of 1,000us = 0.001s
                                                     # 0.001/(1/freq) = pulse width as % of time frame 
                                                     # 4096 ticks per frame; provide value in ticks
                                                     
ESC_PWM_MAX = ESC_PWM_MIN * 2                        # max throttle = 2000us (min x 2, in ticks)


PID_XYZ_ROLL = [[0.030,0.0000,0.000],[0.030,0.0000,0.000],[0,0,0]]
PID_GYRO_ONLY = [[0.0080,0.0000,0.000],[0.0080,0.0000,0.000],[0,0,0]]
PID_XYZ_OFF = [[0,0,0],[0,0,0],[0,0,0]]
PID_AHRS = [[0.01,0.0,0.0],[0.01,0.0,0.0],[0.0,0.0,0.0]]

class Thrust:
    def __init__(self, thrust_limit=1.0):
        if thrust_limit > 1.0 or thrust_limit < 0.0:
            print("WARNING: Thrust limit is out of bounds; disabling thrust", thrust_limit)
            self.thrust_limit = 0.0
        else:
            self.thrust_limit = max(min(thrust_limit, 1.0), 0.0)
        self.esc_range = ESC_PWM_MAX - ESC_PWM_MIN
        self.pwm = Adafruit_PCA9685.PCA9685()  # 16-channel PWM controller
        self.pwm.set_pwm_freq(PWM_FREQ_HZ)
        self.armed = self.arm()
        self.v_pwm = ([ESC_PWM_MIN,ESC_PWM_MIN,ESC_PWM_MIN,ESC_PWM_MIN])
        self.pwm.set_pwm
        self.i = 0
        self.start_time = None
        self.stop_time = None
        
    # setting must be [FORE,STARBOARD,AFT,PORT], each value 0.0 to 100.0
    def set_throttle(self, setting):
        if self.start_time == None:
            self.start_time = time.time()
        
        setting = setting if self.armed else [0,0,0,0]
        self.throttle = np.maximum(setting, 0.0)
        self.throttle = np.minimum(self.throttle, self.thrust_limit)
        
        self.v_pwm[FRONT_RIGHT] = (self.throttle[FORE] + self.throttle[STARBOARD])/2
        self.v_pwm[REAR_RIGHT] =  (self.throttle[AFT]  + self.throttle[STARBOARD])/2
        self.v_pwm[REAR_LEFT] =   (self.throttle[AFT]  + self.throttle[PORT])/2
        self.v_pwm[FRONT_LEFT] =  (self.throttle[FORE] + self.throttle[PORT])/2
        self.v_pwm = np.multiply(self.v_pwm, self.esc_range)
        self.v_pwm = np.rint(self.v_pwm)
        self.v_pwm = np.add(self.v_pwm, ESC_PWM_MIN)
        
        self.pwm.set_pwm(FRONT_RIGHT, 0, int(self.v_pwm[FRONT_RIGHT]))
        self.pwm.set_pwm(REAR_RIGHT,  0, int(self.v_pwm[REAR_RIGHT]))
        self.pwm.set_pwm(REAR_LEFT,   0, int(self.v_pwm[REAR_LEFT]))
        self.pwm.set_pwm(FRONT_LEFT,  0, int(self.v_pwm[FRONT_LEFT]))
        print(self)
        self.i += 1
        self.stop_time = time.time()
    
    def throttle_off(self):
        self.pwm.set_pwm(FRONT_RIGHT, 0, ESC_PWM_MIN)
        self.pwm.set_pwm(REAR_RIGHT,  0, ESC_PWM_MIN)
        self.pwm.set_pwm(REAR_LEFT,   0, ESC_PWM_MIN)
        self.pwm.set_pwm(FRONT_LEFT,  0, ESC_PWM_MIN)
        
    def arm(self):
        print('Arming ESCs')
        for n in range(4):
            self.pwm.set_pwm(FRONT_RIGHT, 0, ESC_PWM_MIN)
        time.sleep(1)
        for n in range(4):
            self.pwm.set_pwm(FRONT_RIGHT, 0, int(ESC_PWM_MIN + self.esc_range * 0.32))
        time.sleep(1)
        for n in range(4):
            self.pwm.set_pwm(FRONT_RIGHT, 0, ESC_PWM_MIN) 
        return True
        
    def disarm(self):
        self.throttle_off()
        self.armed = False
        
    def __str__(self):
        esc_pct = np.subtract(self.v_pwm, ESC_PWM_MIN)
        esc_pct = np.divide(esc_pct, self.esc_range)
        return 'FR,RR,RL,FL' + str(esc_pct) + ', armed = ' + str(self.armed)



'''
3-axis PID controller
 - requires [kp,ki,kd] for each of 3 axes (x,y,z)
 - accepts translation matrix to apply pid output to motor speeds [fore, starboard, aft, port]
 - default translation matrix assumes controller is applied to angular velocity
 - return value from update() is a vector of motor speed adjustments [fore, starboard, aft, port]
'''

class PidController:
    
    t_angular = [[0,-1,0,1],[1,0,-1,0],[-1,1,-1,1]]    # translation matrix to apply angular values to motors
    t_linear =  [[1,0,-1,0],[0,1,0,-1],[-1,-1,-1,-1]]  # translation matrix to apply linear values to motors
    
    def __init__(self, k_3_3, f_state, target=[0.0,0.0,0.0], t=t_angular):
        self.k = k_3_3              # [kp,ki,kd] for each of 3 axes (x,y,z)
        self.f_state = f_state      # function that returns current state (x,y,z)
        self.target = target        # target setpoints for 3 axes (x,y,z)
        self.p = [None,None,None]   # p for 3 axes (x,y,z)
        self.i = [0.0,0.0,0.0]      # i for 3 axes (x,y,z)
        self.d = [0.0,0.0,0.0]      # d for 3 axes (x,y,z)
        self.pid = [0.0,0.0,0.0]    # resulting pid values for 3 axes (x,y,z)
        self.t = t                  # translation matrix to apply pid to motor speeds
        self.count = 0
        self.throttle_adj = [None,None,None,None]
        self.log = []
        self.start_time = None
        self.stop_time = None
        
    def update(self):
        if self.start_time == None:
            self.start_time = time.time()
        self.count += 1
        xyz, xyz_dot, dt = self.f_state()  
        self.throttle_adj = [0.0,0.0,0.0,0.0]
        for n in range(3):
            error = xyz[n] - self.target[n]
            self.p[n] = error
            self.i[n] = xyz_dot[n] * dt
            self.d[n] = -xyz_dot[n]
            self.pid[n] = self.k[n][0] * self.p[n] + self.k[n][1] * self.i[n] + self.k[n][2] * self.d[n]
            self.throttle_adj = np.add(self.throttle_adj, np.multiply(self.pid[n], self.t[n]))
        self.stop_time = time.time()
        return self.throttle_adj
    
    def abs_max(self, v, l):
        return max(min(v,l), -l)
    
    def set_target(self, k_3):
        self.target = k_3
        
    def set_limit(self, limit):
        self.limit = limit
        
    def __str__(self):
        out = str(self.count)
        out += ' p,i,d(x,y,z) = '
        out += str(self.p)
        out += str(self.i)
        out += str(self.d)
        out += ' pid(x,y,z) = '
        out += str(self.pid)
        return out
    
    
class BoundedPid(PidController):
    def __init__(self, k_3_3, f_state, target, t, b_state, b_3):
        super().__init__(k_3_3, f_state, target, t)
        self.b_state = b_state
        self.b_3 = b_3
        
    def update(self, dt):
        self.throttle_adj = super().update(dt)
        vals = self.b_state()
        for n in range(3):
            cap = 1.0 - ((vals[n] - self.b_3[n]) / self.b_3[n])
            self.throttle_adj = np.minimum(self.throttle_adj, cap)
           
        return self.throttle_adj
        
class QuadQav250:
    def __init__(self):
        try:
            self.thrust = Thrust()
            self.ahrs = AHRS()
            self.angular_pid_ahrs = PidController(PID_AHRS,self.ahrs.get)
            self.ahrs.start()
        except Exception:
            self.shutdown()
            print('Failed to initialize')
        
    def shutdown(self):
        try:
            self.thrust.disarm
        finally:
            print('===DISARMED===')
        try:
            self.ahrs.stop()
            self.altimeter.stop()
        finally:
            print('===SHUTDOWN COMPLETE===')
    
    def test(self):
        for n in range(100):
            self.angular_pid_ahrs.update()
            print(self.angular_pid_ahrs)
    
    def kill(self):
        self.shutdown()
    
    def fly(self):
        try:
            print('============================')
            print('STARTING TEST FLIGHT')
            base_throttle = [.32,.32,.32,.32]
            
            t0 = time.time()
            i = 0
            while time.time() < t0 + 6.0:
                pid_update = self.angular_pid_ahrs.update()
                v_throttle = np.add(base_throttle, pid_update)
                self.thrust.set_throttle(base_throttle)
                #print(self.thrust)
                #self.thrust.set_throttle(base_throttle)
            
            print(self.ahrs.i, self.ahrs.stop_time - self.ahrs.start_time, (self.ahrs.stop_time - self.ahrs.start_time)/self.ahrs.i)
            print(self.angular_pid_ahrs.count, self.angular_pid_ahrs.stop_time - self.angular_pid_ahrs.start_time, (self.angular_pid_ahrs.stop_time - self.angular_pid_ahrs.start_time)/self.angular_pid_ahrs.count)  
            print(self.thrust.i, self.thrust.stop_time - self.thrust.start_time, (self.thrust.stop_time - self.thrust.start_time)/self.thrust.i)  
            
        finally:
            self.shutdown()
        
q = QuadQav250()
#q.test()
q.fly()
q.kill()






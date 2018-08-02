import numpy as np
import time
import Adafruit_PCA9685
from nxp_imu import IMU

FRONT_RIGHT = 0
REAR_RIGHT = 1
REAR_LEFT = 2
FRONT_LEFT = 3

FORE = 0
STARBOARD = 1
AFT = 2
PORT = 3
    
ESC_PWM_MAX = 600
ESC_PWM_MIN = 150

PWM_FREQ_HZ = 60

class Thrust:
    def __init__(self, thrust_limit):
        self.thrust_limit = max(min(thrust_limit, 100.0), 0.0)
        self.esc_range = (ESC_PWM_MAX - ESC_PWM_MIN) * (thrust_limit / 100.0) / 100.0
        self.pwm = Adafruit_PCA9685.PCA9685()  # 16-channel PWM controller
        self.pwm.set_pwm_freq(PWM_FREQ_HZ)
        self.armed = True
        self.v_pwm = ([ESC_PWM_MIN,ESC_PWM_MIN,ESC_PWM_MIN,ESC_PWM_MIN])
        self.set_throttle([0,0,0,0]) # FORE, STARBOARD, AFT, PORT
        
    # setting must be [FORE,STARBOARD,AFT,PORT], each value 0.0 to 100.0
    def set_throttle(self, setting):
        # the planar geometric mapping goes here?
        setting = setting if self.armed else [0,0,0,0]
        self.throttle = np.maximum(setting, 0.0)
        self.throttle = np.minimum(self.throttle, 100.0)
        
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
        
    def disarm(self):
        self.set_throttle([0,0,0,0])
        self.armed = False
        
    def __str__(self):
        return str(self.throttle) + str(self.v_pwm) + ', armed = ' + str(self.armed)

class IMU_FXOS8700:
    
    def __init__(self):
        self.imu = IMU()
        
    def get(self):
        return self.imu.get()
        
    def __str__(self):
        return str(self.get())

        
class QuadQav250:
    def __init__(self, thrust_limit = 100.0):
        self.thrust = Thrust(thrust_limit)
        self.imu = IMU_FXOS8700()
        
q = QuadQav250(thrust_limit=80)



q.thrust.set_throttle([100,0,0,0])
time.sleep(1)

q.thrust.set_throttle([0,100,0,0])
time.sleep(1)

q.thrust.set_throttle([0,0,100,0])
time.sleep(1)

q.thrust.set_throttle([0,0,0,100])
time.sleep(1)

q.thrust.disarm()
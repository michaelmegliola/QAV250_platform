import numpy as np
import time
import Adafruit_PCA9685
from Adafruit_BNO055 import BNO055
from nxp_imu import IMU
import VL53L1X
import threading

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

VL53L1X_RANGE_SHORT = 1
VL53L1X_RANGE_MEDIUM = 2
VL53L1X_RANGE_LONG = 3

X = 0
Y = 1
Z = 2

ACCEL = 0
MAG = 1
GYRO = 2

ACCEL_LIMIT = 0.08 # g's

PID_XYZ_EULER = [[1.00,0,0.50],[1.00,0,0.50],[0,0,0]]
PID_XYZ_OFF = [[0,0,0],[0,0,0],[0,0,0]]

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
        
        
    def disarm(self):
        self.set_throttle([0,0,0,0])
        self.armed = False
        
    def __str__(self):
        return str(self.throttle) + str(self.v_pwm) + ', armed = ' + str(self.armed)

class TOF_VL53L1X:  # see https://github.com/pimoroni/vl53l1x-python
    
    def __init__(self):
        self.tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
        self.tof.open()
        self.tof.start_ranging(VL53L1X_RANGE_SHORT)
        self.distance = None
        self.dt = None
        self.velocity = None
        self.i = 0
        self.stopping = False
        
    def calibrate(self):
        print('Calibrating TOF sensor (altimeter)')
        i = 0
        while i < 100:
            d = self.tof.get_distance()
            i += 1
            if d > 50.0:
                print('Cannot calibrate TOF sensor (altimeter); distance = ', d)
                i = 0
            # need visual feedback here
     
    def start(self):
        threading.Thread(target=self.run).start()
        
    def stop(self):
        self.stopping = True
            
    def run(self):
        d0 = self.tof.get_distance()
        t0 = time.time()
        while not self.stopping:
            d1 = self.tof.get_distance()
            t1 = time.time()
            self.distance = d1
            self.dt = t1 - t0
            self.velocity = (d1-d0)/(t1-t0)
            self.i += 1
            d0 = d1
            t0 = t1
  
class AHRS_BNO055:
    def __init__(self):
        self.imu = BNO055.BNO055(serial_port='/dev/serial0', rst=18) # connected to UART, not I2C bus
        self.time = None

    # returns roll, pitch, yaw
    def get_orientation(self):
        y,r,p = self.imu.read_euler()
        t1 = time.time()
        dt = 0 if self.time == None else t1 - self.time
        self.time = t1
        return (r,p,y),dt
    
class IMU_FXOS8700:
    
    def __init__(self):
        self.imu = IMU()
        self.calibrated = False
        self.g_cal = [0.0,0.0,0.0]
        self._z_accel = None
        self.last_a = None
        self.last_g = None
        self.last_ahrs = None
        
    def get(self):
        a, m, g = self.imu.get()
        g = np.subtract(g, self.g_cal)
        return a, m, g
     
    def get_a(self):
        if self.last_a == None:
            dt = 0.0
            self.last_a = time.time()
        else:
            t = time.time()
            dt = t - self.last_a
            self.last_a = t
        return self.get()[ACCEL], dt
        
    def get_g(self):
        if self.last_g == None:
            dt = 0.0
            self.last_g = time.time()
        else:
            t = time.time()
            dt = t - self.last_g
            self.last_g = t
        return self.get()[GYRO], dt
        
    def get_attitude(self):
        if self.last_ahrs == None:
            dt = 0.0
            self.last_ahrs = time.time()
        else:
            t = time.time()
            dt = t - self.last_ahrs
            self.last_ahrs = t
        a, m, g = self.imu.get()
        pitch = (np.arctan2(a[X], np.sqrt(a[Y]**2 + a[Z]**2))*180.0)/np.pi;
        roll = (np.arctan2(-a[Y], a[Z])*180.0)/np.pi;
        return [pitch, roll, 0.0], dt  # heading is unknown
    
    def calibrate(self):
        print('Calibrating gyro...')
        i = 0
        while i < 400:
            a, m, g = self.imu.get()
            self.g_cal = np.add(g, self.g_cal)
            i += 1
            if not self.stationary() or not self.level():
                print('Motion or tilt detected... restarting calibration')
                self.g_cal = [0,0,0]
                i = 0
        self.g_cal = np.divide(self.g_cal, i)
        print('Gyro calibration:', self.g_cal)
                
    def stationary(self):
        a, m, g = self.get()
        if self._z_accel == None:
            self._z_accel = a[X]
            return True
        elif abs(a[X]-self._z_accel) > ACCEL_LIMIT:
            self._z_accel = None
            return False
        else:
            return True
            
    def level(self):
        a, m, g = self.get()
        lim = ACCEL_LIMIT * 3.0
        return abs(a[X]) < lim and abs(a[Y]) < lim and abs(a[Z] - 1.0) < lim
        
    def __str__(self):
        return str(self.get())

'''
3-axis PID controller
 - requires [kp,ki,kd] for each of 3 axes (x,y,z)
 - accepts translation matrix to apply pid output to motor speeds [fore, starboard, aft, port]
 - default translation matrix assumes controller is applied to angular velocity
 - return value from update() is a vector of motor speed adjustments [fore, starboard, aft, port]
'''

class PidController:
    
    t_angular = [[0,-1,0,1],[-1,0,1,0],[-1,1,-1,1]]    # translation matrix to apply angular values to motors
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
        
    def update(self):
        self.count += 1
        vals, dt = self.f_state()
        self.throttle_adj = [0.0,0.0,0.0,0.0]
        for n in range(3):
            error = vals[n] - self.target[n]
            error_dot = 0.0 if self.p[n] == None else (self.p[n] - error) / dt
            self.p[n] = error
            self.i[n] += error * dt
            self.d[n] = -error_dot
            self.pid[n] = self.k[n][0] * self.p[n] + self.k[n][1] * self.i[n] + self.k[n][2] * self.d[n]
            self.throttle_adj = np.add(self.throttle_adj, np.multiply(self.pid[n], self.t[n]))
        return self.throttle_adj
    
    def abs_max(self, v, l):
        return max(min(v,l), -l)
    
    def set_target(self, k_3):
        self.target = k_3
        
    def set_limit(self, limit):
        self.limit = limit
        
    def __str__(self):
        return str(self.count) + '----------------\n' + str(self.p) + '\n' + str(self.i) + '\n' + str(self.d) + '\n' + str(self.pid) + '\n' + str(self.throttle_adj)
    
    
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
    def __init__(self, thrust_limit = 40.0):
        self.thrust = Thrust(thrust_limit)
        self.thrust.set_throttle([0,0,0,0])
        self.altimeter = TOF_VL53L1X()
        #self.altimeter.calibrate()
        #self.altimeter.start()
        self.imu = AHRS_BNO055()
        #self.angular_pid_ahrs = PidController(PID_XYZ_EULER,self.imu.get_orientation)
        self.angular_pid_ahrs = PidController(PID_XYZ_EULER,self.imu.get_orientation)
        
    def shutdown(self):
        self.thrust.disarm()
        self.altimeter.stop()
        print('===SHUTDOWN COMPLETE===')
    
    def test(self):
        for n in range(10):
            self.angular_pid_ahrs.update()
            print(self.angular_pid_ahrs)
    
    def kill(self):
        self.shutdown()
    
    def fly(self):
        try:
            print('============================')
            print('STARTING TEST FLIGHT')
            base_throttle = [0,0,0,0]
            self.thrust.set_throttle(base_throttle)
            t0 = time.time()
            i = 0
            while time.time() < t0 + 4.0:
                self.thrust.set_throttle(np.add(base_throttle, self.angular_pid_ahrs.update()))
                
        finally:
            self.shutdown()
        
q = QuadQav250(thrust_limit=50)
q.fly()
q.kill()




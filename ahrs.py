import numpy as np
import time
import Adafruit_PCA9685
from Adafruit_BNO055 import BNO055
from nxp_imu import IMU
import VL53L1X
import threading

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
        self.imu = BNO055.BNO055(serial_port='/dev/serial0', rst=18) # connected to UART @ 115200 baud, not I2C bus
        self.xyz = (None, None, None)
        self.t0 = None
        self.dt = None
        self.i = 0
        self.stopping = False
        
    def get(self):
        self.i += 1
        y,r,p = self.imu.read_euler()  # TODO: docs say to use quaternions instead of euler angles
        return [r,p,y], time.time()
        
    def start(self, threaded=False):
        if not self.imu.begin():
            raise Exception("IMU failed to initialize")
        self.xyz, self.t0 = self.get()
        if threaded:
            threading.Thread(target=self.run).start() 
             
    def stop(self):
        self.stopping = True
        
    def run(self):
        while not self.stopping:
            self.xyz, t1 = self.get()
            self.dt = t1 - self.t0
            self.t0 = t1
        print("Stopping AHRS_BNO055")
        
    def __str__(self):
        return 'AHRS_BNO055 i=' + str(self.i) + ' (roll,pitch,yaw), dt: ' + str(self.xyz) + ', ' + str(self.dt)
  
class GYRO_FXAS21002:
    def __init__(self):
        self.gyro = IMU(dps=250, gyro_bw=800, verbose=False).gyro
        self.calibrated = False
        self.drift = [0.0,0.0,0.0]
        self.xyz = [0.0,0.0,0.0]
        self.t0 = None
        self.dt = None
        self.i = 0
        self.stopping = False

    def get(self):
        self.i += 1
        x_dot,y_dot,z_dot = self.gyro.get()
        return np.subtract((x_dot,y_dot,z_dot), self.drift), time.time()
        
    def start(self, threaded=False):
        self.calibrate()
        self.xyz, self.t0 = self.get()
        if threaded:
            threading.Thread(target=self.run).start()  
        
    def stop(self):
        self.stopping = True
        
    def run(self):
        while not self.stopping:
            self.xyz, t1 = self.get()
            self.dt = t1 - self.t0
            self.t0 = t1
        print("Stopping GYRO_FXAS21002")
        
    def calibrate(self):
        print('Calibrating GYRO_FXAS21002...')

        for i in range(10):
            self.gyro.get()

        for i in range(400):
            x, y, z = self.gyro.get()
            v = (x,y,z)
            self.drift = np.add(v, self.drift)

        self.drift = np.divide(self.drift, 400)
        print('Gyro calibration:', self.drift)
        
    def __str__(self):
        return 'GYRO_FXAS21002 i=' + str(self.i) + ' (x_dot,y_dot,z_dot), dt: ' + str(self.xyz) + ', ' + str(self.dt)

class AHRS():
    def __init__(self):
        self.gyro = GYRO_FXAS21002()
        self.ahrs = AHRS_BNO055()
        self.i = 0
        self.xyz = [0.0,0.0,0.0]
        self.xyz_dot = [0.0,0.0,0.0]
        self.t0 = None
        self.dt = None
        self.stopping = False
        
    def start(self):
        self.gyro.start()
        self.ahrs.start()
        threading.Thread(target=self.run).start() 
    
    def stop(self):
        self.stopping = True
        self.gyro.stopping = True
        self.ahrs.stopping = True
        
    def run(self):
        self.xyz, self.t0 = self.ahrs.get()
        loop_time = self.t0
        while not self.stopping:
            if time.time() > loop_time + 0.01:
                xyz, t1 = self.ahrs.get()
                self.dt = t1 - self.t0
                self.xyz_dot[0] = (xyz[0] - self.xyz[0]) / self.dt
                self.xyz_dot[1] = (xyz[1] - self.xyz[1]) / self.dt
                self.xyz_dot[2] = (xyz[2] - self.xyz[2]) / self.dt
                self.xyz = xyz
                loop_time = t1
            else:
                self.xyz_dot, t1 = self.gyro.get()
                self.dt = t1 - self.t0
                self.xyz[0] += self.xyz_dot[0] * self.dt
                self.xyz[1] += self.xyz_dot[1] * self.dt
                self.xyz[2] += self.xyz_dot[2] * self.dt
                
            self.t0 = t1
            self.i += 1
            
        print('Stopping AHRS')
    
    def __str__(self):   
        return 'AHRS i=' + str(self.i) + ',' + str(self.ahrs.i) + ',' + str(self.gyro.i) + ' ' + str(self.xyz) + str(self.xyz_dot) + str(self.dt)
            
a = AHRS()
a.start()
i = 0
while a.ahrs.i < 3:
    print(a)
    i = a.ahrs.i
a.stop()
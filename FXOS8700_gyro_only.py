# From https://github.com/MomsFriendlyRobotCompany/nxp_imu
#!/usr/bin/env python

from nxp_imu import IMU
import time

"""
accel/mag - 0x1f
gyro - 0x21
pi@r2d2 nxp $ sudo i2cdetect -y 1
	0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 1f
20: -- 21 -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
"""

bandwidths = [25, 50, 100, 200, 400, 800]

def gyro_test():

	imu = IMU(dps=250, gyro_bw=800, verbose=False)
	gyro = imu.gyro
	
	for _ in range(10):
		print(gyro.get())
		
	t = time.time()
	for _ in range(1000):
		gyro.get()
	print(time.time()-t)

if __name__ == "__main__":
	try:
		gyro_test()
	except Exception as e:
		print(e)
	except KeyboardInterrupt:
		pass

	print('Done ...')

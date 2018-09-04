import logging
import sys
import time
import math

from Adafruit_BNO055 import BNO055

'''
FROM ADAFRUIT

Why doesn't Euler output seem to match the Quaternion output?

The Euler angles coming out of the chip are based on 'automatic orientation 
detection', which has the drawback of not being continuous for all angles and 
situations. According to Bosch BNO055 Euler angle output should only be used 
for eCompass, where pitch and roll stay below 45 degrees.

For absolute orientation, quaternions should always be used, and they can be 
converted to Euler angles at the last moment via the .toEuler() helper function 
in quaternion.h.
'''

def quaternion_to_euler_angle(w, x, y, z):
	
	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + y * y)
	X = math.degrees(math.atan2(t0, t1))
	
	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y = math.degrees(math.asin(t2))
	
	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (y * y + z * z)
	Z = math.degrees(math.atan2(t3, t4))
	
	return X, Y, Z


bno = BNO055.BNO055(serial_port='/dev/serial0', rst=18)
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

# Print system status and self test result.
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')

# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = bno.get_revision()
print('Software version:   {0}'.format(sw))
print('Bootloader version: {0}'.format(bl))
print('Accelerometer ID:   0x{0:02X}'.format(accel))
print('Magnetometer ID:    0x{0:02X}'.format(mag))
print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

print('Reading BNO055 data, press Ctrl-C to quit...')


while True:
    z,x,y = bno.read_euler()
    print('xyz','{:>8,f}'.format(x),'{:>8,f}'.format(y),'{:>8,f}'.format(z))
    time.sleep(.1)


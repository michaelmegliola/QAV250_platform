import time
import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685()

esc_min = 150  # throttle 0
esc_max = 320  # throttle 30%

pwm.set_pwm_freq(60)

for i in range(4):
    pwm.set_pwm(i, 0, esc_min) #arm ESCs

for i in range(4):
    print('Setting throttle to 0%'.format(esc_min))
    pwm.set_pwm(i, 0, esc_min)

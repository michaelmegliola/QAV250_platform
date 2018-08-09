import time
import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685()

esc_min = 150  # throttle 0
esc_max = 380  # throttle 30%

pwm.set_pwm_freq(60)

for i in range(4):
    pwm.set_pwm(i, 0, esc_min) #arm ESCs


for i in range(4):
    print('Setting throttle to 0%'.format(esc_min))
    pwm.set_pwm(i, 0, esc_min)
    time.sleep(1)
    print('Setting throttle to 100%'.format(esc_max))
    pwm.set_pwm(i, 0, esc_max)
    time.sleep(1)
    print('Setting throttle to 0%'.format(esc_min))
    pwm.set_pwm(i, 0, esc_min)
    time.sleep(1)



import time
import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685()

esc_min = 246  # throttle 0
esc_max = 492  # throttle 100%
esc_target = int((esc_max-esc_min) * 0.30) + esc_min

print(esc_target)

pwm.set_pwm_freq(60)

for i in range(4):
    pwm.set_pwm(i, 0, esc_min) #arm ESCs


for i in range(4):
    print('Setting throttle to 0%'.format(esc_min))
    pwm.set_pwm(i, 0, esc_min)
    time.sleep(1)
    print('Setting throttle to 100%'.format(esc_max))
    pwm.set_pwm(i, 0, esc_target)
    time.sleep(1)
    print('Setting throttle to 0%'.format(esc_min))
    pwm.set_pwm(i, 0, esc_min)
    time.sleep(1)



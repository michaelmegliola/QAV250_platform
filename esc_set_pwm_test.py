import time
import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685()

esc_min = 246  # throttle 0
esc_max = 492  # throttle 100%
esc_target = int((esc_max-esc_min) * 0.30) + esc_min

pwm.set_pwm_freq(60)

for i in range(4):
    pwm.set_pwm(i, 0, esc_min) #arm ESCs

t0 = time.time()

for j in range(1000):
    for i in range(4):
        pwm.set_pwm(i, 0, esc_target)
        
t1 = time.time()

print((t1-t0)/1000)



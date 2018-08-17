import time
import Adafruit_PCA9685

esc = Adafruit_PCA9685.PCA9685()

esc_pwm_freq = 50  # PWM frequency used to communicate with the ESCs
tick_length = (1000000 / esc_pwm_freq) / 4096  # 1 second in us, divided by the freq, divided by the bit depth is the tick length per us
esc_min = 1000 * tick_length  # 1000us in ticks
esc_max = 2000 * tick_length  # 2000us in ticks

print('throttle off signal: ', esc_min)



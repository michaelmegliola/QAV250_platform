# QAV250_platform
Test code, setup, other docs for our 250-class quadcopter platform

The platform is based on a commodity 250-class quad kit (racing drone).  Instead of a dedicated flight computer, we mounted a Raspberry Pi Zero W, with the following accessories connected via I2C:
- an Adafruit (or clone) PCA9685 based PWM driver board
- an Adafruit Precision NXP 9-DOF Breakout Board (IMU)
- a Sparkfun (or clone) VL53L1X-based laser ToF sensor (rangefinder)

The platform will allow for code development, specifically flight control software.  The sensors will allow for flight based on angular position, angular velocity, and distance from the ground.

We recommend following the steps in the guide to set up the OS, Python, Python modules for all of the components, and Cloud9 for Python dev via a web interface.

We are also providing a 3D model of a replacement top plate for the quadcopter frame, which allows for easy mounting of the components above.  This may not be compatible with your quad, but you're welcome to use it.  If you want to use the same frame as us, we used this: https://www.gearbest.com/goods/pp_009249661790.html  However, please note that this was an arbitrary choice and we are not endorsing or recommending any particular frame.

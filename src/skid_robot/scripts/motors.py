#!/usr/bin/env python
from board import SCL, SDA
import busio

# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685

# Create the I2C bus interface.
i2c_bus = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c_bus, address=0x40)

# Set the PWM frequency to 60hz.
pca.frequency = 60

pwm = pca.channels[4]
fwd = pca.channels[6]
rev = pca.channels[5]

fwd.duty_cycle = 0xFFFF
rev.duty_cycle = 0x0000
pwm.duty_cycle = 0

#if __name__ == '__main__':
#    listener()

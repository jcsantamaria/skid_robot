#!/usr/bin/env python
from __future__ import division

# Import the PCA9685 module.
import Adafruit_PCA9685

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Set the PWM frequency to 60hz.
pwm.set_pwm_freq(60)

mot = 4
fwd = 6
rev = 5

def duty_cycle(channel, cycle):
    off = int(cycle / 0xFFFF * 4095)
    pwm.set_pwm(channel, 0, off)
    print("channel: %d on: %d off: %d" % (channel, 0, off))


duty_cycle(fwd, 0xFFFF)
duty_cycle(rev, 0x0000)
duty_cycle(mot, 0)

#if __name__ == '__main__':
#    listener()

#!/usr/bin/env python
from __future__ import division

# Import the PCA9685 module.
import Adafruit_PCA9685

# Constants
FRONT_MAX_RPM = 1600
REAR_MAX_RPM = 2160


class Motor:
    def __init__(self, pca, pwm, fwd, rev):
        self._pca = pca
        self._pwm = pwm
        self._fwd = fwd
        self._rev = rev

        self.stop()

    def stop(self):
        self._pca.set_pwm(self._fwd, 0, 4096)
        self._pca.set_pwm(self._rev, 0, 4096)
        self._pca.set_pwm(self._pwm, 0, 4096)

    def move(self, speed):
        f = True
        r = False
        if speed > 0:
            s = max(0, min(int(speed * 4095), 4095))
        else:
            s = max(0, min(int(-speed * 4095), 4095))
            f = False
            r = True
        if f:
            self._pca.set_pwm(self._fwd, 4096, 0)
        else:
            self._pca.set_pwm(self._fwd, 0, 4096)
        if r:
            self._pca.set_pwm(self._rev, 4096, 0)
        else:
            self._pca.set_pwm(self._rev, 0, 4096)

        self._pca.set_pwm(self._pwm, 0, s)
        # print('pulses = {0}'.format(s))


class DriveTrain:

    def __init__(self, flpwm, flfwd, flrev, frpwm, frfwd, frrev, blpwm, blfwd, blrev, brpwm, brfwd, brrev, address=0x40, frequency=60):
        # Initialise the PCA9685 using the default address (0x40).
        self.pca = Adafruit_PCA9685.PCA9685(address=address)
        # Set the PWM frequency
        self.pca.set_pwm_freq(frequency)

        # define motors
        self.fl = Motor(self.pca, flpwm, flfwd, flrev)
        self.fr = Motor(self.pca, frpwm, frfwd, frrev)
        self.bl = Motor(self.pca, blpwm, blfwd, blrev)
        self.br = Motor(self.pca, brpwm, brfwd, brrev)

        # define wheelbase: 9in = 0.2286m
        self.wheelBase = 0.2286
        # define wheelRadius: 2.5in = 0.0635m
        self.wheelRadius = 0.0635

    @property
    def FrontLeft(self):
        return self.fl

    @property
    def FrontRight(self):
        return self.fr

    @property
    def RearLeft(self):
        return self.bl

    @property
    def RearRight(self):
        return self.br

    @property
    def WheelBase(self):
        return self.wheelBase

    @WheelBase.setter
    def WheelBase(self, wheelBase):
        if wheelBase < 0.1:
            raise ValueError("Out of range")
        self.wheelBase = wheelBase

    def stop(self):
        self.fl.stop()
        self.fr.stop()
        self.bl.stop()
        self.br.stop()

    def move(self, left, right):
        self.fl.move(left * 60 / FRONT_MAX_RPM)
        self.bl.move(left * 60 / REAR_MAX_RPM)
        self.fr.move(right * 60 / FRONT_MAX_RPM)
        self.br.move(right * 60 / REAR_MAX_RPM)

    def drive(self, linear, angular):
        # skeed steer kinematics
        left = (linear - angular * self.wheelBase / 2.0) / self.wheelRadius
        right = (linear + angular * self.wheelBase / 2.0) / self.wheelRadius
        self.move(left, right)

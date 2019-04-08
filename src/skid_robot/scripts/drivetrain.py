#!/usr/bin/env python
from board import SCL, SDA
import busio

# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685

class Motor:
    def __init__(self, pwm, fwd, rev):
        self._pwm = pwm
        self._fwd = fwd
        self._rev = rev

        self.stop()

    def stop(self):
        self._fwd.duty_cycle = 0x0000
        self._rev.duty_cycle = 0x0000
        self._pwm.duty_cycle = 0x0000

    def move(self, speed):
        f = 0xFFFF
        r = 0x0000
        if speed > 0:
            s = max(0, min(int(speed * 0xFFFF / 100), 0xFFFF))
        else:
            s =  max(0, min(int(-speed * 0xFFFF / 100), 0xFFFF))
            f = 0x0000
            r = 0xFFFF
        self._fwd.duty_cycle = f
        self._rev.duty_cycle = r
        self._pwm.duty_cycle = s
        #print("pulses = %0X" % s)


class DriveTrain:

    def __init__(self, flpwm, flfwd, flrev, frpwm, frfwd, frrev, blpwm, blfwd, blrev, brpwm, brfwd, brrev, address = 0x40, frequency = 60):
        # Create the I2C bus interface.
        self.i2c_bus = busio.I2C(SCL, SDA)
        # Create a simple PCA9685 class instance.
        self.pca = PCA9685(self.i2c_bus, address=address)
        # Set the PWM frequency
        self.pca.frequency = frequency

        #define motors
        self.fl = Motor(self.pca.channels[flpwm], self.pca.channels[flfwd], self.pca.channels[flrev])
        self.fr = Motor(self.pca.channels[frpwm], self.pca.channels[frfwd], self.pca.channels[frrev])
        self.bl = Motor(self.pca.channels[blpwm], self.pca.channels[blfwd], self.pca.channels[blrev])
        self.br = Motor(self.pca.channels[brpwm], self.pca.channels[brfwd], self.pca.channels[brrev])

        #define wheelbase: 9in = 0.2286m
        self.wheelBase = 0.2286
        #define wheelRadius: 2.5in = 0.0635m
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
    def WheelBase(self, wheelbase):
        if wheelBase < 0.1:
            raise ValueError("Out of range")
        self.wheelBase = wheelBase

    def stop(self):
        self.fl.stop()
        self.fr.stop()
        self.bl.stop()
        self.br.stop()

    def move(self, left, right):
        self.fl.move(left)
        self.bl.move(left)
        self.fr.move(right)
        self.br.move(right)

    def drive(self, linear, angular):
        # skeed steer kinematics
        left  = (linear - angular * self.wheelBase / 2.0) / self.wheelRadius
        right = (linear + angular * self.wheelBase / 2.0) / self.wheelRadius
        self.move(left,right)
        
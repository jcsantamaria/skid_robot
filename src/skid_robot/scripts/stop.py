#!/usr/bin/env python
from drivetrain import DriveTrain

#import board as _
#print(_.__file__)

train = DriveTrain(4, 6, 5, 10, 12, 11, 9, 7, 8, 15, 14, 13)

train.FrontLeft.stop()
train.FrontRight.stop()
train.RearLeft.stop()
train.RearRight.stop()

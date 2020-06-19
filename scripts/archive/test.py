#!/usr/bin/env python
from drivetrain import DriveTrain

train = DriveTrain(4, 6, 5, 10, 12, 11, 9, 7, 8, 15, 14, 13)

# train.FrontLeft.move(50)
# train.FrontRight.move(25)
# train.RearLeft.move(-25)
# train.RearRight.move(-50)
train.drive(0.01, 0)

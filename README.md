Shooter side is front of robot
Origin of robot is the center of the swerve modules (same as CAD)


CAN ID List:
Power Distribution Hub - 13
Pigeon 2.0 - 14
--
Front Left Drive - 1
Front Left Steer - 2
Front Left Encoder - 9
--
Front Right Drive - 3
Front Right Steer - 4
Front Right Encoder - 10
--
Rear Left Drive - 5
Rear Left Steer - 6
Rear Left Encoder - 11
--
Rear Right Drive - 7
Rear Right Steer - 8
Rear Right Encoder - 12
--
Wrist Leader (Left) - 15
Wrist Follower (Right)- 16
--
Intake - 17
Feeder Lower - 18
Feeder Upper - 19
--
Shooter Lower (Left) - 20
Shooter Upper (Right)- 21


Sensor IO:
Object Beam Break 1 - DIO 0
Object Beam Break 2 (Unused) - DIO 1
Wrist Limit Switch - DIO 2

Cameras:
--
Model - 3G
Name - intake
Position - (0,0,0)



Questions for Jason:
Do we need the advantageScope Swerve Calibration.json



Alert Types:
Error - Critical functions will not work. You should never start a match if able.
Warning - Non-critical functions have issues. Robot will continue to work, but some functions may be unavailable. You should not start a match with warnings
Info - Used more for debugging on the practice field. Information about robot condition that may be useful to know, but do not impact robot operation.

CAN ID Groups:
1 - 19 / Swerve
20 - 39 / Devbot
40 - 59 / Compbot
60 - 64 / Spare

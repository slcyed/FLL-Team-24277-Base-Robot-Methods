# LEGO type:standard slot:0 autostart

import base_robot
import sys
from spike.control import wait_for_seconds, wait_until, Timer
from spike.operator import greater_than, greater_than_or_equal_to, \
    less_than, less_than_or_equal_to, equal_to, not_equal_to

br = base_robot.BaseRobot()
br.hub.motion_sensor.reset_yaw_angle()
br.AccelGyroDriveForward(60)
br.TurnRightAndDriveOnHeading(60, 90)
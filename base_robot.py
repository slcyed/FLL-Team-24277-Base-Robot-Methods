from spike import PrimeHub, LightMatrix, Button, StatusLight, \
    ForceSensor, MotionSensor, Speaker, ColorSensor, App, \
    DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from spike.operator import greater_than, greater_than_or_equal_to, \
    less_than, less_than_or_equal_to, equal_to, not_equal_to
import math
import sys

class BaseRobot():
    """
    A collection of methods and Spike Prime objects for FLL Team 24277. \
    The BaseRobot class has two drive motors as a MotorPair, two medium \
    motors for moving attachments, and all of the base methods available for \
    Spike Prime sensors and motors. It also includes some custom methods \
    for moving the robot. Enjoy!

    Example:

    >>> import base_robot
    >>> import sys
    >>> br = base_robot.BaseRobot()
    >>> br.AccelGyroDriveForward(40)
    >>> br.GyroTurn(90)
    """
    def __init__(self):
        self.hub = PrimeHub()
        self._version = "1.2 8/4/2022"
        self._leftDriveMotorPort = 'E'
        self._rightDriveMotorPort = 'A'
        self._leftAttachmentMotorPort = 'B'
        self._rightAttachmentMotorPort = 'D'
        self._colorSensorPort = 'F'
        self.driveMotors = MotorPair(self._leftDriveMotorPort, self._rightDriveMotorPort)
        self.debugMode = False
        self._tireDiameter = 5.6 #cm
        self._tireCircum = self._tireDiameter * math.pi #cm

    

    def GyroTurn(self, angle):
        """
        Turns the robot the specified number of `desiredDegrees`. 
        Positive numbers turn to the right, negative numbers turn the robot \
            to the left. Note that when the robot makes the turn, it will \
            always overshoot by about seven degrees. In other words if you \
            need a +90 degree turn, you will probably end up commanding \
            something around +83 degrees. You may also want to put a \
            wait_for_seconds(0.2) or something like that after a gyro turn. \
            Just to make sure the robot has stopped moving before continuing \
            with more instructions.
        Parameter
        -------------
        angle: How many degrees should the robot turn. \
            Positive values turn the robot to the right, negative values turn \
            to the left.
        type: float
        values: Any. Best to keep the numbers less than 180, just so the \
            robot doesn't turn more than necessary.
        default: No default value
        """
        gyroTurnSpeed = 5
        MotionSensor().reset_yaw_angle()
        
        if(angle > 0):
            while(MotionSensor().get_yaw_angle() < angle):
                self.driveMotors.start_tank(gyroTurnSpeed, -gyroTurnSpeed)
        else:
            while(MotionSensor().get_yaw_angle() > angle):
                self.driveMotors.start_tank(-gyroTurnSpeed, gyroTurnSpeed)
        self.driveMotors.stop()
    
    
    def GyroDriveOnHeading(self, distance, heading):
        """
        Moves the robot the specified distance.

        Parameters:
        -----------
        
        Heading


        Distance

        >>> import base_robot
        >>> br = base_robot.BaseRobot()
        >>> br.GyroDriveOnHeading(20, 45)
        """
        maxSpeed = 50
        minSpeed = 10
        proportionFactor = 1
        
        totalDegreesNeeded = distance / self._tireCircum * 360
        MotionSensor().reset_yaw_angle()
        testmotor = Motor(self._leftDriveMotorPort)
        testmotor.set_degrees_counted(0)
        
        #Accel to full speed
        for currentSpeed in range(0, maxSpeed, 5):
            correction = self.hub.motion_sensor.get_yaw_angle() - heading
            self.driveMotors.start(steering = correction * proportionFactor, speed = currentSpeed)
            wait_for_seconds(0.1)
        
        #Cruise at full speed
        slowDownPoint = totalDegreesNeeded - 360
        while(testmotor.get_degrees_counted() < slowDownPoint):
            correction = self.hub.motion_sensor.get_yaw_angle() - heading
            self.driveMotors.start(steering = correction * proportionFactor, speed = maxSpeed)
        
        #Slow down
        for currentSpeed in range(maxSpeed, minSpeed, -5):
            correction = self.hub.motion_sensor.get_yaw_angle() - heading
            self.driveMotors.start(steering = correction * proportionFactor, speed = currentSpeed)
            wait_for_seconds(0.1)
            
        #Stop
        self.driveMotors.stop()
    
    def AccelGyroDriveForward(self, distance):
        """
        Drives the robot very straight for `desiredDistance`, using \
            acceleration and gyro.
        
        Accelerates to prevent wheel slipping. Gyro keeps the robot \
        pointing on the same heading.
        Minimum distance that this will work for is about 16cm. \
        If you need to go a very short distance, use ``move_tank``.
        Parameters
        ----------
        desiredDistance: How far the robot should go in cm
        type: float
        values: Any value above 16.0. You can enter smaller numbers, but the \
            robot will still go 16cm
        default: No default value
        Example
        -------
        >>> import base_robot
        >>> br = base_robot.BaseRobot()
        >>> br.AccelGyroDriveForward(20)
        """
        self.GyroDriveOnHeading(distance, self.hub.motion_sensor.get_yaw_angle())

    def TurnRightAndDriveOnHeading(self, distance, heading):
        """
        Turns the robot to the right until the `heading` \
        is reached. Then drives on the `heading` until \
        the `distance` has been reached.
        Minimum distance that this will work for is about 16cm. \
        If you need to go a very short distance, use ``GyroTurn`` and \
        move_tank.
        Parameters
        ----------
        heading: On what heading should the robot drive
        type: float
        values: any. However, it must be a heading larger than the current \
            heading (that is, to the right). If a heading is entered that is \
            less than the current heading, the program will exit. default: no \
            default value
        distance: How far the robot should go in cm
        type: float
        values: any value above 16.0. You can enter smaller numbers, but the \
            robot will still go 16cm
        default: no default value
        Example
        -------
        >>> import base_robot
        >>> br = base_robot.BaseRobot()
        >>> br.TurnRightAndDriveOnHeading(90, 40) #drive heading 90 for 40 cm
        """
        if heading < self.hub.motion_sensor.get_yaw_angle & self.debugMode==True():
            sys.exit("TurnRightAndDriveOnHeading Error: Invalid Heading, try using TurnLeftAndDriveOnHeading Method")
        
        self.GyroTurn(heading - self.hub.motion_sensor.get_yaw_angle())
        self.GyroDriveOnHeading(distance, heading)

    def TurnLeftAndDriveOnHeading(self, distance, heading):
        """
        Turns the robot to the left until the `heading` \
        is reached. Then drives on the `heading` until \
        the `distance` has been reached.
        Minimum distance that this will work for is about 16cm. \
        If you need to go a very short distance, use ``GyroTurn`` and \
        move_tank.
        Parameters
        ----------
        heading: On what heading should the robot drive
        type: float
        values: any. However, it must be a heading larger than the current \
            heading (that is, to the left). If a heading is entered that is \
            less than the current heading, the program will exit. default: no \
            default value
        distance: How far the robot should go in cm
        type: float
        values: any value above 16.0. You can enter smaller numbers, but the \
            robot will still go 16cm
        default: no default value
        Example
        -------
        >>> import base_robot
        >>> br = base_robot.BaseRobot()
        >>> br.TurnLeftAndDriveOnHeading(90, 40) #drive heading 90 for 40 cm
        """
        #Debug
        if heading > self.hub.motion_sensor.get_yaw_angle & self.debugMode==True():
            sys.exit("TurnLeftAndDriveOnHeading Error: Invalid Heading, try using TurnRightAndDriveOnHeading Method")
        
        #Turns Left
        self.GyroTurn(self.hub.motion_sensor.get_yaw_angle() - heading)
        #Drives on selected Heading
        self.GyroDriveOnHeading(distance, heading)
    
    def GetVersion(self):
        return self._version
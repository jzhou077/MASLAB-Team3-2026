from raven import Raven

import math
import time

from localization.pose import Pose
from math.vector import Vector
from localization.localizer import Localizer
from utils.robot_timer import Timer
from math.motion_profile import MotionProfile
from purepursuit.path import Path, PathPoint
from purepursuit.calculators import ErrorCalculator, VectorCalculator
from purepursuit.bezier_stuff import BezierCurve

import utils.constants as constants
import utils.utils as utils

class Robot:
    
    def __init__(self, startPose=Pose()):
        self.raven_board = Raven()
        self.initialize_motors()

        self.localizer = Localizer(self.raven_board, startPose)
        self.mp = MotionProfile()

        self.current_pose = startPose.copy()
        self.closestPose = PathPoint(0, Pose(), Vector())
        self.previousClosestPose = PathPoint(0, Pose(), Vector())
        self.current_path = None

        self.targetPoint = None
        self.holdingPosition = False
        self.isTurning = False
        self.isBusy = False

        self.path_timer = Timer()
        self.zeroVelocityDetectedTimer = None

        self.errorCalculator = ErrorCalculator()
        self.vectorCalculator = VectorCalculator()

        self.max_power_scaling = 1.0

        self.servo_position = -40

        # this is a little confusing, right might be 2*pi - left.theta?? 34.2
        self.left_vector = Vector(magnitude=Pose(8, 0, 0).cartesianToPolar()[0], theta=Pose(10, 0, 0).cartesianToPolar()[1]).normalize()
        self.right_vector = Vector(magnitude=Pose(8, 0, 0).cartesianToPolar()[0], theta=Pose(10, 0, 0).cartesianToPolar()[1]).normalize()
        self.vectors = [self.left_vector, self.right_vector]

    def initialize_motors(self):
        self.left_motor = Raven.MotorChannel.CH4
        self.right_motor = Raven.MotorChannel.CH1
        self.servo = Raven.ServoChannel.CH1

        self.left_intake = Raven.MotorChannel.CH2
        self.right_intake = Raven.MotorChannel.CH3

        self.raven_board.set_motor_mode(self.left_intake, Raven.MotorMode.DIRECT)
        self.raven_board.set_motor_mode(self.right_intake, Raven.MotorMode.DIRECT)

        self.raven_board.set_motor_mode(self.left_motor, Raven.MotorMode.DIRECT)
        self.raven_board.set_motor_mode(self.right_motor, Raven.MotorMode.DIRECT)    
        self.raven_board.set_motor_torque_factor(self.left_motor, 100)
        self.raven_board.set_motor_torque_factor(self.right_motor, 100)
        self.raven_board.set_motor_max_current(self.left_motor, 4)
        self.raven_board.set_motor_max_current(self.right_motor, 4)

    def update(self):
        self.update_pose()

        if self.current_path is None:
            return

        # FINISH LATER
        if self.holdingPosition:
            self.previousClosestPose = self.closestPose
            self.closestPose = self.current_path.updateClosestPose(self.localizer.get_pose(), 1)
            self.updateErrorsAndVectors()
            self.run_drive(self.vectorCalculator.get_diff_drive_powers())
            if abs(self.errorCalculator.getHeadingError() < 0.75 and self.isTurning):
                self.isTurning = False
                self.isBusy = False
            return

        if self.isBusy:
            self.previousClosestPose = self.closestPose
            self.closestPose = self.current_path.updateClosestPose(self.localizer.get_pose(), 10) # 10 should be default
            self.updateErrorsAndVectors()
            self.run_drive(self.vectorCalculator.get_diff_drive_powers())

        if self.localizer.get_velocity().getAsVector().magnitude < 1.0 and self.current_path.getClosestPoint().tValue > 0.8 and self.zeroVelocityDetectedTimer is None and self.isBusy:
            self.zeroVelocityDetectedTimer = Timer()
        
        if not (self.current_path.isAtParametricEnd() or self.zeroVelocityDetectedTimer is not None and self.zeroVelocityDetectedTimer.getElapsedTime() > 500.0):
            return

    def update_pose(self):
        self.localizer.update()
        self.current_pose = self.localizer.get_pose()
        self.current_left_ticks, self.current_right_ticks = self.localizer.get_encoder_ticks()

    def updateErrors(self):
        print(self.closestPose.tangentVector.theta)
        self.errorCalculator.update(self.current_pose, self.current_path, self.closestPose, self.localizer.current_velocity.getAsVector(), 34.2, self.closestPose.tangentVector.theta if not self.isTurning else self.current_path.getLastControlPoint().getHeading()) #34.2 being the forward vel of the bot

    def updateVectors(self):
        # update takes in max power, centripetal scaling, current pose, closest pose, velocity, current path, drive error, heading error, translational error, heading goal
        # centripetal scaling might need to be tuned
        self.vectorCalculator.update(1.0, 1.0, self.current_pose, self.closestPose, self.localizer.current_velocity.getAsVector(), self.current_path, self.errorCalculator.getDriveError() if not self.holdingPosition else -1, self.errorCalculator.getHeadingError(), self.errorCalculator.getTranslationalError(), self.closestPose.tangentVector.theta)

    def updateErrorsAndVectors(self):
        self.updateErrors()
        self.updateVectors()

    def follow_path(self, path, holdEnd):
        self.break_following()
        self.holdPositionAtEnd = holdEnd
        self.isBusy = True
        self.setPath(path)
        self.previousClosestPose = self.closestPose
        self.closestPose = self.current_path.updateClosestPose(self.localizer.get_pose(), 10)

    def setPath(self, path):
        self.current_path = path
        self.current_path.init()

    def break_following(self):
        self.errorCalculator.break_following()
        self.vectorCalculator.breakFollowing()
        self.holdingPosition = False
        self.isBusy = False
        self.reachedEnd = False
        self.zeroVelocityDetectedTimer = None

    def print_coordinates(self):
        print(f"X: {self.current_pose.x}, Y: {self.current_pose.y}, Heading: {self.current_pose.heading}")

    
    def run_drive(self, drivePowers):
        drivePowers[0] = utils.clamp(drivePowers[0], -1, 1)
        drivePowers[1] = utils.clamp(drivePowers[1], -1, 1)

        self.raven_board.set_motor_mode(self.left_motor, Raven.MotorMode.DIRECT)
        self.raven_board.set_motor_mode(self.right_motor, Raven.MotorMode.DIRECT)    
        self.raven_board.set_motor_torque_factor(self.left_motor, 100)
        self.raven_board.set_motor_torque_factor(self.right_motor, 100)
        self.raven_board.set_motor_max_current(self.left_motor, 4)
        self.raven_board.set_motor_max_current(self.right_motor, 4)
        
        if drivePowers[0] < 0:
            self.raven_board.set_motor_speed_factor(self.left_motor, -int(drivePowers[0] * 100), reverse=False)
        else:
            self.raven_board.set_motor_speed_factor(self.left_motor, int(drivePowers[0] * 100), reverse=True)
        
        if drivePowers[1] < 0:
            self.raven_board.set_motor_speed_factor(self.right_motor, -int(drivePowers[1] * 100), reverse=True)
        else:
            self.raven_board.set_motor_speed_factor(self.right_motor, int(drivePowers[1] * 100), reverse=False)

    def turn(self, angleDeg, isLeft):
        temp = Pose(self.current_pose.getX(), self.current_pose.getY(), self.current_pose.getHeading() + (angleDeg if isLeft else -angleDeg))
        self.hold_point(Path(BezierCurve([temp, temp, temp])), False)
        self.isTurning = True
        self.isBusy = True

    def turnToDegrees(self, deg):
        temp = Pose(self.current_pose.getX(), self.current_pose.getY(), deg)
        self.hold_point(Path(BezierCurve([temp, temp, temp])), False)
        self.isTurning = True
        self.isBusy = True

    def hold_point(self, pose, heading):
        self.break_following()
        self.holdingPosition = True
        self.isBusy = False
        self.setPath(pose)
        self.previousClosestPose = self.closestPose
        self.closestPose = self.current_path.updateClosestPose(self.localizer.get_pose(), 1)
    
    def servo_to_start(self):
        self.raven_board.set_servo_position(self.servo, 40, 800, 2200)
        self.servo_position = 55

    def intake(self, speed):
        self.raven_board.set_motor_max_current(self.left_intake, 4, 10)
        self.raven_board.set_motor_max_current(self.right_intake, 4, 10)
        self.raven_board.set_motor_mode(self.left_intake, Raven.MotorMode.DIRECT)
        self.raven_board.set_motor_mode(self.right_intake, Raven.MotorMode.DIRECT)
        # raven_board.set_motor_pid(self.left_intake, p_gain=12, i_gain=0, d_gain=0.2, percent = 100, retry=10)
        # raven_board.set_motor_pid(self.right_intake, p_gain=12, i_gain=0, d_gain=0.2, percent = 100, retry=10)
        if speed < 0:
            speed = abs(speed)
            self.raven_board.set_motor_speed_factor(self.left_intake, speed, reverse = True, retry=10)       
            self.raven_board.set_motor_speed_factor(self.right_intake, speed, reverse = False, retry=10)
        else:
            self.raven_board.set_motor_speed_factor(self.left_intake, speed, reverse = False, retry=10)
            self.raven_board.set_motor_speed_factor(self.right_intake, speed, reverse = True, retry=10)
    
    def servo_off(self):
        self.raven_board.set_servo_off(self.servo)

    def move_servo(self, position):
        # upright position = -55
        # down position = 55
        while position < self.servo_position:
            self.raven_board.set_servo_position(self.servo, self.servo_position - 1, 800, 2200)
            self.servo_position -= 1    
            time.sleep(0.05)
            print("Servo Position:", self.raven_board.get_servo_position(self.servo, 800, 2200))
        while position > self.servo_position:
            self.raven_board.set_servo_position(self.servo, self.servo_position + 1, 800, 2200)
            self.servo_position += 1
            time.sleep(0.05)
            print("Servo Position:", self.raven_board.get_servo_position(self.servo, 800, 2200))

        self.raven_board.set_servo_off(self.servo)

    def stack_cans(self):

        self.move_servo(0)
        self.raven_board.set_servo_position(self.servo, 0, 800, 2200)
        time.sleep(0.5)
        self.intake(0)
        time.sleep(1)
        
        self.move_servo(-42)
        time.sleep(2)
        self.raven_board.set_servo_position(self.servo, -45, 800, 2200)
        time.sleep(1)
        self.outtake()

        # for i in range(-11, -18, -1):
        #     self.intake(i)
        #     if i == -17:
        #         self.move_to_position_basic(self.x - 4 * math.cos(self.heading), self.y - 4 * math.sin(self.heading), timeout=2, max_speed=13, reverse=True)
        #         raven_board.set_servo_position(self.servo, -39, 800, 2200)

        #     time.sleep(1.8)


        self.intake(0)
        self.raven_board.set_servo_off(self.servo)

    def outtake(self, timeDifferenceBetween=0.075):
        self.raven_board.set_motor_mode(self.left_intake, Raven.MotorMode.POSITION)
        self.raven_board.set_motor_mode(self.right_intake, Raven.MotorMode.POSITION)
        self.raven_board.set_motor_pid(self.left_intake, 4, 0, 0.1, 100)
        self.raven_board.set_motor_pid(self.right_intake, 4, 0, 0.1, 100)
        
        startLTicks = self.raven_board.get_motor_encoder(self.left_intake)
        startRTicks = self.raven_board.get_motor_encoder(self.right_intake)

        lticks = self.raven_board.get_motor_encoder(self.left_intake)
        rticks = self.raven_board.get_motor_encoder(self.right_intake)

        while abs(lticks - startLTicks) < 3200:
            lticks = self.raven_board.get_motor_encoder(self.left_intake)
            rticks = self.raven_board.get_motor_encoder(self.right_intake)

            self.raven_board.set_motor_target(self.left_intake, lticks - 200)
            self.raven_board.set_motor_target(self.right_intake, rticks + 200)

            time.sleep(timeDifferenceBetween*2.5)

            # lticks = raven_board.get_motor_encoder(self.left_intake)
            # rticks = raven_board.get_motor_encoder(self.right_intake)

            # raven_board.set_motor_target(self.left_intake, lticks + 250)
            # raven_board.set_motor_target(self.right_intake, rticks - 250)

            # time.sleep(timeDifferenceBetween*2)

        self.move_to_position_basic(self.x - 4 * math.cos(self.heading), self.y - 4 * math.sin(self.heading), timeout=2, max_speed=13, reverse=True)
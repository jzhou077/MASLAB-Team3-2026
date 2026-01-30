from raven import Raven

import math

from localization.pose import Pose
from localization.localizer import Localizer
from utils.robot_timer import Timer
from math.motion_profile import MotionProfile
import utils.constants as constants
import utils.utils as utils

class Robot:
    
    def __init__(self, startPose=Pose()):
        self.raven_board = Raven()
        self.left_motor = Raven.MotorChannel.CH5
        self.right_motor = Raven.MotorChannel.CH4

        self.raven_board.set_motor_mode(self.left_motor, Raven.MotorMode.POSITION)
        self.raven_board.set_motor_mode(self.right_motor, Raven.MotorMode.POSITION)    
        self.raven_board.set_motor_torque_factor(self.left_motor, 100)
        self.raven_board.set_motor_torque_factor(self.right_motor, 100)
        self.raven_board.set_motor_max_current(self.left_motor, 4)
        self.raven_board.set_motor_max_current(self.right_motor, 4)

        self.localizer = Localizer(self.raven_board, startPose)
        self.mp = MotionProfile()

        self.current_pose = startPose.copy()
        self.start_pose = startPose.copy()
        self.current_left_ticks = 0
        self.current_right_ticks = 0
        self.start_left_ticks = 0
        self.start_right_ticks = 0

        self.targetPoint = None
        self.holdingPosition = False
        self.isTurning = False
        self.isBusy = False

        self.path_timer = Timer()
        self.zeroVelocityDetectedTimer = None

    def update(self):
        self.update_pose()

        if self.targetPoint == None:
            return
        
        #fix later cuz, should be after isbusy
        if self.holdingPosition:
            self.raven_board.set_motor_target(self.left_motor, self.current_left_ticks)
            self.raven_board.set_motor_target(self.right_motor, self.current_right_ticks)

            self.isBusy = False
            
            return
        
        if self.isBusy:
            print("Busy")
            if self.current_pose.distanceFrom(self.targetPoint) < constants.DISTANCE_THRESHOLD_INCHES:
                angle_error = utils.normalize_angle_degrees(self.current_pose.getHeading() - self.targetPoint.getHeading())
            else:
                angle_error = utils.normalize_angle_degrees(self.current_pose.angleBetween(self.targetPoint))
            print("Target Heading: " + str(self.targetPoint.getHeading()))
            print("Angle Error: " + str(angle_error))
            
            if not self.isTurning and abs(angle_error) > constants.ANGLE_THRESHOLD_DEGREES:
                self.isTurning = True
                return
            
            elif self.isTurning:
                if abs(angle_error) < constants.ANGLE_THRESHOLD_DEGREES:
                    self.isTurning = False

                target_ticks_for_heading = (angle_error / 360) * constants.TICKS_PER_CENTER_REVOLUTION
                self.raven_board.set_motor_pid(self.left_motor, 3, 0, 0.2, 60)
                self.raven_board.set_motor_pid(self.right_motor, 3, 0, 0.2, 60)

                self.raven_board.set_motor_target(self.left_motor, self.current_left_ticks - target_ticks_for_heading)
                self.raven_board.set_motor_target(self.right_motor, self.current_right_ticks - target_ticks_for_heading)

            else:
                target_ticks_for_path = self.mp.run(self.start_pose.distanceFrom(self.targetPoint),
                                    self.path_timer.getElapsedTime(),
                                    34.2,
                                    68.4) / constants.INCHES_PER_TICK                
                # print(self.current_pose.distanceFrom(self.targetPoint))
                print(f"X: {self.current_pose.x}, Y: {self.current_pose.y}, Heading: {self.current_pose.heading}")
                print(f"Target Point --- X: {self.targetPoint.x}, Y: {self.targetPoint.y}, Heading: {self.targetPoint.heading}")
                self.raven_board.set_motor_pid(self.left_motor, p_gain=20, i_gain=0, d_gain=2, percent=100)
                self.raven_board.set_motor_pid(self.right_motor, p_gain=20, i_gain=0, d_gain=2, percent=100)
                self.raven_board.set_motor_target(self.left_motor, self.start_left_ticks - target_ticks_for_path)
                self.raven_board.set_motor_target(self.right_motor, self.start_right_ticks + target_ticks_for_path)

            percentage_traveled = 1 - (self.current_pose.distanceFrom(self.targetPoint) / (self.startPoint.distanceFrom(self.targetPoint) + 0.000000000001))
            velocity = self.localizer.get_velocity()
            speed = math.hypot(velocity.x, velocity.y)

            print("Percentage Traveled: " + str(percentage_traveled))
            print("Speed: " +  str(speed))
            
            if speed < 1.0 and percentage_traveled > 0.8 and self.zeroVelocityDetectedTimer == None and self.isBusy:
                self.zeroVelocityDetectedTimer = Timer()

            # Should we not change this to the distance threshold?
            if ((percentage_traveled > 0.995 and abs(angle_error) < 1) or (self.zeroVelocityDetectedTimer != None and self.zeroVelocityDetectedTimer.getElapsedTime() > 5)):
                self.break_following()

            if self.path_timer.getElapsedTime() < 0.025:    
                print("Resetting start pose and ticks")
                self.start_pose = self.current_pose.copy()
                self.start_left_ticks = self.current_left_ticks
                self.start_right_ticks = self.current_right_ticks

    def update_pose(self):
        self.localizer.update()
        self.current_pose = self.localizer.get_pose()
        self.current_left_ticks, self.current_right_ticks = self.localizer.get_encoder_ticks()

    def move_to_point(self, point: Pose, holdEnd=False):
        if point == self.targetPoint:
            return

        self.break_following()
        self.holdPositionAtEnd = holdEnd
        self.isBusy = True
        self.path_timer.reset()
        self.startPoint = self.current_pose.copy()
        self.targetPoint = point

    def break_following(self):
        self.holdingPosition = False
        self.isBusy = False
        self.reachedEnd = False
        self.zeroVelocityDetectedTimer = None

    def print_coordinates(self):
        print(f"X: {self.current_pose.x}, Y: {self.current_pose.y}, Heading: {self.current_pose.heading}")
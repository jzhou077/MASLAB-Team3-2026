from raven import Raven

import math
import time

from pose import Pose
from localization.imu import IMU
from utils.robot_timer import Timer, NanoTimer
import utils.constants as constants

class Localizer:

    def __init__(self, raven_board, start_pose=Pose()):
        self.raven_board = raven_board
        self.imu = IMU(start_pose.getHeading())

        self.previous_left_encoder_ticks = 0
        self.previous_right_encoder_ticks = 0
        self.left_encoder_ticks = 0
        self.right_encoder_ticks = 0

        self.heading = start_pose.getHeading()
        self.previous_heading = start_pose.getHeading()

        self.start_pose = start_pose
        self.timer = Timer()
        self.delta_time = 0.00001
        self.displacement_pose = Pose()
        self.current_velocity = Pose()
        self.total_heading = 0

        self.previousPoseTime = time.time()
        self.currentPoseTime = time.time()

    def update(self):
        self.delta_time = self.timer.getElapsedTime()
        self.timer.reset()

        self.previousPoseTime = self.currentPoseTime
        self.currentPoseTime = time.time()

        self.update_trackers()

        delta_left_inches = (-1 if constants.LEFT_MOTOR_REVERSED else 1) * (self.left_encoder_ticks - self.previous_left_encoder_ticks) * constants.INCHES_PER_TICK
        delta_right_inches = (-1 if constants.RIGHT_MOTOR_REVERSED else 1) * (self.right_encoder_ticks - self.previous_right_encoder_ticks) * constants.INCHES_PER_TICK
        delta_center_inches = (delta_left_inches + delta_right_inches) / 2
        
        # Delta heading could be wrong depending on if my logic is right
        delta_heading = (self.heading - self.previous_heading)
        # print("Delta Heading: " + str(delta_heading))

        delta_x = delta_center_inches * math.cos(math.radians(self.heading))
        delta_y = delta_center_inches * math.sin(math.radians(self.heading))
        self.displacement_pose = self.displacement_pose + Pose(delta_x, delta_y, delta_heading)
        self.current_velocity = Pose(delta_x/self.delta_time, delta_y/self.delta_time, delta_heading/self.delta_time)

        self.total_heading += delta_heading

        # print(self.displacement_pose)

    def update_trackers(self):
        self.previous_left_encoder_ticks = self.left_encoder_ticks
        self.previous_right_encoder_ticks = self.right_encoder_ticks

        self.left_encoder_ticks = self.raven_board.get_motor_encoder(Raven.MotorChannel.CH4)
        self.right_encoder_ticks = self.raven_board.get_motor_encoder(Raven.MotorChannel.CH1)

        self.previous_heading = self.heading
        self.heading = self.imu.find_adjusted_heading()
        # print("Current Heading: " + str(self.heading))

    def get_pose(self):
        return self.start_pose + self.displacement_pose
    
    def get_velocity(self):
        return self.current_velocity
    
    # sets current pose estimate, changing should only change robot's current pose estimate
    def set_pose(self, set_pose):
        self.displacement_pose = set_pose - self.start_pose
        self.reset_encoders()

    def reset_encoders(self):
        self.raven_board.set_motor_encoder(Raven.MotorChannel.CH4, 0)
        self.raven_board.set_motor_encoder(Raven.MotorChannel.CH1, 0)

    def resetIMU(self):
        self.imu.reset()

    def get_encoder_ticks(self):
        return self.left_encoder_ticks, self.right_encoder_ticks
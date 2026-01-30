import time

from pose import Pose
from localization.localizer import Localizer

class PoseTracker:

    def __init__(self, raven_board):
        self.startingPose = Pose(0, 0, 0)
        self.currentPose = self.startingPose.copy()
        self.previousPose = self.startingPose.copy()
        self.currentVelocity = Pose()
        self.previousVelocity = Pose()
        self.currentAcceleration = Pose()

        self.previousPoseTime = 0
        self.currentPoseTime = 0

        self.localizer = Localizer(raven_board)

    def update(self):
        self.previousVelocity = self.getVelocity()
        self.previousPose = self.getPose()
        self.currentPose = None
        self.currentVelocity = None
        self.currentAcceleration = None
        self.previousPoseTime = self.currentPoseTime
        self.currentPoseTime = time.time()
        self.localizer.update()
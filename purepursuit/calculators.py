import math

import utils.utils as utils
import purepursuit.kinematics as kinematics
from purepursuit.controls import KalmanFilter, PIDFController, FilteredPIDFController
from math.vector import Vector
from pose import Pose

class ErrorCalculator:
    def __init__(self):
        self.driveKalmanFilter = KalmanFilter(6, 1)
        self.velocityVector = Vector()

        self.currentPose = None
        self.currentPath = None
        self.closestPose = None
        self.xVelocity = None
        self.rawDriveError = None
        self.previousRawDriveError = None
        self.headingGoal = None
        self.driveErrors = [0, 0]
        self.driveError = None

    def update(self, currentPose, currentPath, closestPose, velocity, xMovement, headingGoal):
        self.currentPose = currentPose
        self.velocityVector = velocity
        self.currentPath = currentPath
        self.closestPose = closestPose
        self.xVelocity = xMovement
        self.headingGoal = headingGoal
        self.driveError = None

    def getTranslationalError(self):
        error = Vector()
        x = self.closestPose.pose.getX() - self.currentPose.getX()
        y = self.closestPose.pose.getY() - self.currentPose.getY()
        error.setOrthogonalComponents(x, y)
        return error

    def getHeadingError(self):
        if self.currentPath == None:
            return 0
        
        headingError = utils.getTurnDirection(self.currentPose.getHeading(), self.headingGoal) * utils.getSmallestAngleDifference(self.currentPose.getHeading(), self.headingGoal)
        print("Heading Error: " + str(headingError))
        return headingError

    def getDriveVelocityError(self, distanceToGoal):
        if self.currentPath is None:
            return 0

        forwardZeroPowerAcceleration = -17 # -34 is just some random value i picked for our deceleration, probably need to tune or something

        tangent = self.currentPath.closestPointTangentVector
        distanceToGoalVector = tangent.times(distanceToGoal)
        velocity = self.velocityVector.projectOnto(tangent)

        forwardHeadingVector = Vector(magnitude=1.0, theta=self.currentPose.getHeading())
        forwardVelocity = forwardHeadingVector.dot(velocity)
        forwardDistanceToGoal = forwardHeadingVector.dot(distanceToGoalVector)
        forwardVelocityGoal = kinematics.getVelocityToStopWithDeceleration(forwardDistanceToGoal, forwardZeroPowerAcceleration * 2) 
        forwardVelocityZeroPowerDecay = forwardVelocity - kinematics.getFinalVelocityAtDistance(forwardVelocity, forwardZeroPowerAcceleration, forwardDistanceToGoal)

        forwardVelocityError = Vector(magnitude=forwardVelocityGoal - forwardVelocityZeroPowerDecay - forwardVelocity, theta=forwardHeadingVector.theta)

        self.previousRawDriveError = self.rawDriveError
        self.rawDriveError = forwardVelocityError.magnitude * utils.signum(forwardVelocityError.dot(tangent))

        projection = kinematics.predictNextLoopVelocity(self.driveErrors[1], self.driveErrors[0])

        self.driveKalmanFilter.update(self.rawDriveError - self.previousRawDriveError, projection)

        for i in range(len(self.driveErrors)-1):
            self.driveErrors[i] = self.driveErrors[i + 1]
        
        self.driveErrors[1] = self.driveKalmanFilter.state

        return self.driveKalmanFilter.state

    def getDriveError(self):
        if self.driveError is not None:
            return self.driveError
        
        if self.currentPath is None:
            return 0

        if not self.currentPath.isAtParametricEnd():
            distanceToGoal = self.currentPath.getDistanceRemaining()
        else:
            offset = (self.currentPath.getLastControlPoint() - self.currentPose).getAsVector()
            distanceToGoal = self.currentPath.getEndTangent().dot(offset)

        driveError = self.getDriveVelocityError(distanceToGoal)

        return driveError
    
    def break_following(self):
        self.driveError = 0.0
        self.headingError = 0
        self.rawDriveError = 0
        self.previousRawDriveError = 0
        self.driveErrors = [0.0, 0.0]
        self.driveKalmanFilter.reset(0, 1, 1)

class VectorCalculator:
    def __init__(self):
        # need to put in values
        self.drivePIDF = FilteredPIDFController(0.0005, 0, 0.00002, 0, 0) #p, i, d, f, t
        self.secondaryDrivePIDF = FilteredPIDFController(0.002, 0, 0.00004, 0, 0)
        self.headingPIDF = PIDFController(0.275, 0, 0.005, 0) #p: 0.35, i, d: 0.005, f
        self.secondaryHeadingPIDF = PIDFController(0.8, 0, 0.0025, 0) # 1.5, 0, 0.0015, 0

        self.drivePIDFSwitch = 20
        self.headingPIDFSwitch = 5

        self.currentPath = None
        self.currentPose, self.closestPose = None, None
        self.headingError, driveError = None, None
        self.headingGoal = None

        self.velocities = list()
        self.accelerations = list()
        self.velocity = Vector()

        self.averageVelocity, self.averagePreviousVelocity, self.averageAcceleration = Vector(), Vector(), Vector()
        self.translationalIntegralVector, self.secondaryTranslationalIntegralVector = Vector(), Vector()

        self.driveVector, self.headingVector, self.translationalVector, self.centripetalVector, self.correctiveVector, self.translationalError = Vector(), Vector(), Vector(), Vector(), Vector(), Vector()

        self.previousSecondaryTranslationalIntegral = 0.0
        self.previousTranslationalIntegral = 0.0

        self.maxPowerScaling = 1.0
        self.mass = 10 # NEED TO ADJUST THIS
        
        self.scaleDriveFeedForward = False
        self.centripetalScaling = 0.0

        self.k_cte = 0.03         # cross-track to turn
        self.k_heading = 0.03
        
    def update(self, maxPowerScaling, centripetalScaling, currentPose, closestPose, velocity, currentPath, driveError, headingError, translationalError, headingGoal):
        self.maxPowerScaling = maxPowerScaling
        self.centripetalScaling = centripetalScaling
        self.currentPose = currentPose
        self.closestPose = closestPose
        self.velocity = velocity
        self.currentPath = currentPath
        self.driveError = driveError
        self.headingError = headingError
        self.translationalError = translationalError
        self.headingGoal = headingGoal

    def breakFollowing(self):
        self.drivePIDF.reset()
        self.headingPIDF.reset()
        
        self.driveVector = Vector()
        self.headingVector = Vector()

    def getDriveVector(self):
        if self.driveError == -1:
            self._v_cmd = self.maxPowerScaling
            return Vector(magnitude=self.maxPowerScaling, theta=self.currentPath.closestPointTangentVector.theta)

        tangent = self.currentPath.closestPointTangentVector.normalize()

        self.drivePIDF.updateFeedForwardInput(utils.signum(self.driveError))
        self.drivePIDF.updateError(self.driveError)

        self._v_cmd = utils.clamp(self.drivePIDF.run(), -self.maxPowerScaling, self.maxPowerScaling)

        # vector is for visualization / mecanum style, but for diff drive use _v_cmd
        self.driveVector = Vector(magnitude=abs(self._v_cmd),
                                theta=tangent.theta if self._v_cmd >= 0 else tangent.theta + math.pi)
        return self.driveVector.copy()
    
    def getHeadingVector(self):
        self.headingPIDF.updateFeedForwardInput(utils.getTurnDirection(self.currentPose.getHeading(), self.headingGoal))
        self.headingPIDF.updateError(self.headingError)
        self.headingVector = Vector(magnitude=utils.clamp(self.headingPIDF.run(), -self.maxPowerScaling, self.maxPowerScaling), theta=self.currentPose.getHeading())
        return self.headingVector.copy()
    
    def _signed_cross_track_error(self) -> float:
        """
        Signed cross-track error using the path normal at the closest point.
        Positive means robot is on the +normal side of the path.
        """
        # error vector from robot to closest point (world frame)
        # ErrorCalculator.getTranslationalError returns (closest - robot)
        e = self.translationalError.copy()

        # Use the normal vector you already compute in Path.updateClosestPose()
        # It should be a unit-ish vector pointing to one side of the path.
        n = self.currentPath.closestPointNormalVector.normalize()

        # signed distance (in your distance units)
        return e.dot(n)

    def get_diff_drive_powers(self):
        """
        Returns (left, right) wheel powers for a 2-wheel differential drive.
        Assumes: drivePID gives forward command, curvature + errors give turn command.
        """
        max_p = self.maxPowerScaling

        self.drivePIDF.updateFeedForwardInput(utils.signum(self.driveError))
        self.drivePIDF.updateError(self.driveError)
        self.secondaryDrivePIDF.updateError(self.driveError)
        if abs(self.driveError) > self.drivePIDFSwitch:
            v = utils.clamp(self.drivePIDF.run(), -max_p, max_p)
        else:
            v = utils.clamp(self.secondaryDrivePIDF.run(), -max_p, max_p)

        self.headingPIDF.updateFeedForwardInput(utils.getTurnDirection(self.currentPose.getHeading(), self.headingGoal))
        self.headingPIDF.updateError(self.headingError)
        self.secondaryHeadingPIDF.updateError(self.headingError)
        if abs(self.headingError) > 5:
            omega_heading = utils.clamp(self.k_heading * self.headingPIDF.run(), -max_p, max_p)
        else:
            omega_heading = utils.clamp(self.k_heading * self.secondaryHeadingPIDF.run(), -max_p, max_p)
        

        cte = self._signed_cross_track_error()
        omega_cte = utils.clamp(self.k_cte * (cte / 12.0), -max_p, max_p)  # if inches

        omega = utils.clamp(omega_heading + omega_cte, -max_p, max_p)
        left = v - omega
        right = v + omega

        # normalize if needed
        m = max(abs(left), abs(right))
        if m > max_p:
            left = (left / m) * max_p
            right = (right / m) * max_p

        # print(f"v={v:.3f} omega_h={omega_heading:.3f} omega_cte={omega_cte:.3f} omega_ff={omega_ff:.3f} -> left={left:.3f} right={right:.3f} cte={cte:.3f} curv={curvature:.3f}")

        return [left, right]
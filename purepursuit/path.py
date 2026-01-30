from purepursuit.bezier_stuff import BezierCurve
from localization.pose import Pose

class Path:
    def __init__(self, curve):
        self.curve = curve

        self.closestPointCurvature = 0
        self.closestPointTValue = 0
        self.closestPose = None
        self.closestPointTangentVector = None

    def getPose(self, t):
        position = self.curve.get_pose(t)
        return Pose(position.getX(), position.getY(), position.getHeading())

    def getTangentVector(self, t):
        return self.curve.get_derivative(t)

    def getClosestPoint(self, pose=None, searchLimit=None):
        if pose is None and searchLimit is None:
            return PathPoint(self.closestPointTValue, self.closestPose, self.closestPointTangentVector)

        initialTValueGuess = self.curve.get_closest_point(pose, searchLimit, self.closestPointTValue)
        return self.getPoseInformation(initialTValueGuess)

    def getPoseInformation(self, t):
        return PathPoint(t, self.getPose(t), self.getTangentVector(t))

    def updateClosestPose(self, currentPose, searchLimit):
        self.closestPoint = self.getClosestPoint(currentPose, searchLimit)
        self.closestPointTValue = self.closestPoint.tValue
        self.closestPose = self.closestPoint.pose
        self.closestPointTangentVector = self.closestPoint.tangentVector
        self.closestPointNormalVector = self.curve.get_normal_vector(self.closestPointTValue)
        self.closestPointCurvature = self.curve.get_curvature(self.closestPointTValue)
        print(f"X: {self.closestPoint.pose.x}, Y: {self.closestPoint.pose.y}")
        return self.closestPoint

    def isAtParametricEnd(self):
        return self.curve.at_parametric_end(self.closestPointTValue)

    def isAtParametricStart(self):
        return self.closestPointTValue <= 0.005

    def getDistanceRemaining(self, t=None):
        if t is not None:
            return self.curve.length - (self.curve.get_path_completion(t) * self.curve.length)
        
        return self.curve.length - (self.curve.get_path_completion(self.closestPointTValue) * self.curve.length)

    def getLastControlPoint(self):
        return self.curve.control_points[-1]

    def getEndTangent(self):
        return self.curve.end_tangent
    
    def init(self):
        self.curve.initialize()

class PathPoint:
    def __init__(self, tValue, pose, tangentVector):
        self.tValue = tValue
        self.pose = pose
        self.tangentVector = tangentVector
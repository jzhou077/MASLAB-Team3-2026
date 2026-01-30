import utils.utils as utils
import math

class Vector:
    def __init__(self, pose=None, magnitude=None, theta=None):
        if pose == None and magnitude == None and theta == None:
            self.setComponents(0,0)
        elif pose is not None:
            self.setOrthogonalComponents(pose.getX(), pose.getY())
        else:
            self.setComponents(magnitude, theta)

    def setComponents(self, magnitude, theta):
        if magnitude < 0:
            self.magnitude = -magnitude
            self.theta = utils.normalize_angle(theta + math.degrees(math.pi))
        else:
            self.magnitude = magnitude
            self.theta = utils.normalize_angle(theta)

        self.xComponent, self.yComponent = self.polarToCartesian(magnitude, theta)

    def setOrthogonalComponents(self, xComponent, yComponent):
        self.xComponent = xComponent
        self.yComponent = yComponent

        self.magnitude, self.theta = self.cartesianToPolar(xComponent, yComponent)

    def rotateVector(self, theta2):
        self.setTheta(self.theta + theta2)

    def setTheta(self, theta):
        self.setComponents(self.magnitude, theta)

    def polarToCartesian(self, r, theta):
        return (r * math.cos(math.radians(theta)), r * math.sin(math.radians(theta)))

    def cartesianToPolar(self, x, y):
        return (math.hypot(x, y), utils.normalize_angle(math.degrees(math.atan2(y, x))))

    def minus(self, other):
        returnVector = Vector()
        returnVector.setOrthogonalComponents(self.xComponent - other.xComponent, self.yComponent - other.yComponent)
        return returnVector
    
    def plus(self, other):
        returnVector = Vector()
        returnVector.setOrthogonalComponents(self.xComponent + other.xComponent, self.yComponent + other.yComponent)
        return returnVector

    def times(self, scalar):
        return Vector(magnitude=self.magnitude * scalar, theta=self.theta)

    def dot(self, other):
        return self.xComponent * other.xComponent + self.yComponent * other.yComponent

    def cross(self, other):
        return self.xComponent * other.yComponent - self.yComponent * other.xComponent

    def projectOnto(self, other):
        if other.magnitude == 0:
            return Vector()

        scale = self.dot(other) / other.dot(other)
        return other.times(scale)
    
    def normalize(self):
        if self.magnitude == 0:
            return Vector(magnitude=0.0, theta=self.theta)
        else:
            return Vector(magnitude=self.magnitude / abs(self.magnitude), theta=self.theta)

    def copy(self):
        return Vector(magnitude=self.magnitude, theta=self.theta)
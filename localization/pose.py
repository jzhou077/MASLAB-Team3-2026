import math
import numpy as np
from math.vector import Vector
import utils.utils as utils

class Pose:
    def __init__(self, x = 0.0, y = 0.0, heading = 0.0):
        self.x = x
        self.y = y
        self.heading = heading
    
    def getX(self):
        return self.x

    def getY(self):
        return self.y

    def getHeading(self):
        return self.heading

    def distanceFrom(self, other):
        return math.sqrt((other.x - self.x)**2 + (other.y - self.y)**2)

    def angleBetween(self, other):
        # theta = arccos((a*b)/(|a||b|))
        return self.heading - math.degrees(math.atan2(other.y - self.y, other.x - self.x))

    def setX(self, x):
        self.x = x

    def setY(self, y):
        self.y = y

    def setHeading(self, heading):
        self.heading = heading

    def copy(self):
        return Pose(self.x, self.y, self.heading)

    def distSquared(self, other):
        xDist = other.x - self.x
        yDist = other.x - self.y
        return xDist**2 + yDist**2

    def getAsVector(self):
        vector = Vector()
        vector.setOrthogonalComponents(self.x, self.y)
        return vector

    def cartesianToPolar(self):
        return (math.hypot(self.x, self.y), utils.normalize_angle(math.atan2(self.y, self.x)))
    
    def __add__(self, other):
        return Pose(self.x + other.x, self.y + other.y, self.heading + other.heading)
    
    def __sub__(self, other):
        return Pose(self.x - other.x, self.y - other.y, self.heading - other.heading)

    def __eq__(self, other):
        if other == None:
            return False
        return self.x == other.x and self.y == other.y and self.heading == other.heading
    
    def __str__(self):
        return f"Pose(x: {self.x}, y: {self.y}, heading: {self.heading})"
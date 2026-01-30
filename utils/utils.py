import math
import utils.constants as constants

# Normalize angle to be within [-pi, pi]
def normalize_angle_rad(angle):
    return (angle - math.pi) % (2 * math.pi) - math.pi

def normalize_angle_degrees(angle):
    return (angle - 180) % 360 - 180

# Wrap angle to be within [0, 2*pi]
def wrap_angle(angle):
    return angle % (2 * math.pi)

def to_ticks(val_in_inches):
    return val_in_inches * 3200/(math.pi * constants.WHEEL_DIAMETER_INCHES)

def normalize_angle(angleDeg):
    angle = angleDeg % 360
    if angle < 0:
        return angle + 360

    return angle

def getTurnDirection(startHeading, endHeading):
    if normalize_angle(endHeading - startHeading) >= 0 and normalize_angle(endHeading - startHeading) <= 180:
        return 1 # CCW
    return -1 # CW

def getSmallestAngleDifference(one, two):
    return min(normalize_angle(one - two), normalize_angle(two - one))

def signum(x):
    return (x > 0) - (x < 0)

def clamp(val, minimum, maximum):
    if val > maximum:
        val = maximum
    if val < minimum:
        val = minimum
    
    return val

def findNormalizingScaling(staticVector, variableVector, maxPowerScaling):
    a = variableVector.xComponent ** 2 + variableVector.yComponent ** 2
    b = staticVector.xComponent * variableVector.xComponent + staticVector.yComponent * variableVector.yComponent
    c = staticVector.xComponent ** 2 + staticVector.yComponent ** 2 - maxPowerScaling ** 2
    return (-b + math.sqrt(b**2 - a*c))/(a)
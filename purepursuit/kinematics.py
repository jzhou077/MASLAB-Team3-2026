import math

import utils.utils as utils

def getVelocityToStopWithDeceleration(directionalDistance, deceleration):
    return utils.signum(directionalDistance) * math.sqrt(abs(-2 * deceleration * directionalDistance))

def getFinalVelocityAtDistance(currentVelocity, deceleration, directionalDistance):
    return utils.signum(directionalDistance) * math.sqrt(abs(currentVelocity**2 + 2 * deceleration * abs(directionalDistance)))

def predictNextLoopVelocity(currentVelocity, previousVelocity):
    return 2 * currentVelocity - previousVelocity
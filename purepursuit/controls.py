import time

class KalmanFilter:
    def __init__(self, modelCovariance, dataCovariance):
        self.modelCovariance = modelCovariance
        self.dataCovariance = dataCovariance

        self.reset(0, 1, 1)

    def reset(self, startState, startVariance, startGain):
        self.state = startState
        self.previousState = startState
        self.variance = startVariance
        self.previousVariance = startVariance
        self.kalmanGain = startGain 

    def update(self, updateData, updateProjection):
        self.state = self.previousState + updateData
        self.variance = self.previousVariance + self.modelCovariance
        self.kalmanGain = self.variance / (self.variance + self.dataCovariance)
        self.state += self.kalmanGain * (updateProjection - self.state)
        self.variance *= (1.0 - self.kalmanGain)
        self.previousState = self.state
        self.previousVariance = self.variance

class PIDFController:
    def __init__(self, p_gain, i_gain, d_gain, f):
        self.p = p_gain
        self.i = i_gain
        self.d = d_gain
        self.f = f

        self.feedFowardInput = 0.0

        self.reset()

    def run(self):
        return (self.error * self.p) + (self.errorDerivative * self.d) + (self.errorIntegral * self.i) + (self.feedFowardInput * self.f)

    def updatePosition(self, position):
        self.position = position
        self.previousError = self.error
        self.error = self.targetPosition - self.position

        deltaTimeNano = time.time_ns() - self.previousUpdateTimeNano
        self.previousUpdateTimeNano = time.time_ns()

        self.errorIntegral += self.error * (deltaTimeNano / 10**9)
        self.errorDerivative = (self.error - self.previousError) / (deltaTimeNano / 10**9)

    def updateError(self, error):
        self.previousError = self.error
        self.error = error

        deltaTimeNano = time.time_ns() - self.previousUpdateTimeNano
        self.previousUpdateTimeNano = time.time_ns()

        self.errorIntegral += self.error * (deltaTimeNano / 10**9)
        self.errorDerivative = (self.error - self.previousError) / (deltaTimeNano / 10**9)

    def updateFeedForwardInput(self, inp):
        self.feedFowardInput = inp

    def reset(self):
        self.previousError = 0
        self.error = 0
        self.position = 0
        self.targetPosition = 0
        self.errorIntegral = 0
        self.errorDerivative = 0
        self.previousUpdateTimeNano = time.time_ns()

    def setTargetPosition(self, target):
        self.targetPosition = target

class FilteredPIDFController:
    def __init__(self, p_gain, i_gain, d_gain, f, t):
        self.p = p_gain
        self.i = i_gain
        self.d = d_gain
        self.f = f
        self.t = t

        self.feedFowardInput = 0.0

        self.reset()

    def run(self):
        return (self.error * self.p) + (self.filteredDerivative * self.d) + (self.errorIntegral * self.i) + self.f * self.feedFowardInput
    
    def updatePosition(self, update):
        self.position = update
        self.previousError = self.error
        self.error = self.targetPosition - self.position

        deltaTimeNano = time.time_ns() - self.previousUpdateTimeNano
        self.previousUpdateTimeNano = time.time_ns()

        self.errorIntegral += self.error * (deltaTimeNano / 10**9)
        self.previousDerivative = self.filteredDerivative
        self.errorDerivative = (self.error - self.previousError) / (deltaTimeNano / 10**9)
        self.filteredDerivative = self.t * self.previousDerivative + (1 - self.t) * self.errorDerivative
    
    def updateError(self, error):
        self.previousError = self.error
        self.error = error

        deltaTimeNano = time.time_ns() - self.previousUpdateTimeNano
        self.previousUpdateTimeNano = time.time_ns()

        self.errorIntegral += self.error * (deltaTimeNano / 10**9)
        self.previousDerivative = self.errorDerivative
        self.errorDerivative = (self.error - self.previousError) / (deltaTimeNano / 10**9)
        self.filteredDerivative = self.t * self.previousDerivative + (1 - self.t) * self.errorDerivative
    
    def updateFeedForwardInput(self, inp):
        self.feedFowardInput = inp

    def setTargetPosition(self, target):
        self.targetPosition = target

    def reset(self):
        self.previousError = 0
        self.error = 0
        self.position = 0
        self.targetPosition = 0
        self.errorIntegral = 0
        self.errorDerivative = 0
        self.previousDerivative = 0
        self.filteredDerivative = 0
        self.previousUpdateTimeNano = time.time_ns()
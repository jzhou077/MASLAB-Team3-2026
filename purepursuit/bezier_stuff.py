from math.matrix import Matrix
from localization.pose import Pose 
from math.vector import Vector
import purepursuit.characteristic_matrix_supplier as characteristic_matrix_supplier
import utils.utils as utils

import math
import bisect

class BezierCurve:
    APPROXIMATION_STEPS = 1000

    def __init__(self, control_points):
        self.control_points = control_points
        self.end_tangent = Vector()

        self.initialized = False
        self.length = None
        self.cached_matrix = Matrix()

        self.diff_powers = None
        self.diff_coefficients = None

        self.completion_map = {}

        self.initialize()

    def initialize(self):
        # if already initialized, do nothing
        if self.initialized:
            return

        self.initialized = True
        self.generate_bezier_curve()
        self.length = self.approximate_length()
        self.end_tangent.setOrthogonalComponents(self.control_points[-1].getX() - self.control_points[-2].getX(), self.control_points[-1].getY() - self.control_points[-2].getY())
        self.end_tangent = self.end_tangent.normalize()

    def approximate_length(self):
        previousPoint = self.get_pose(0)
        currentPoint = None
        approxLength = 0

        for i in range(1, 1000): # 1000 approximation steps
            t = float(i)/float(1000)
            currentPoint = self.get_pose(t)
            approxLength += previousPoint.distanceFrom(currentPoint)
            previousPoint = currentPoint
            self.completion_map[t] = approxLength
        
        return approxLength

    def generate_bezier_curve(self):
        controlPointMatrix = Matrix(len(self.control_points), 2)

        for i in range(len(self.control_points)):
            p = self.control_points[i] # will be a Pose object
            controlPointMatrix.set_row(i, [p.getX(), p.getY()])

        self.cached_matrix = (characteristic_matrix_supplier.get_bezier_characteristic_matrix(len(self.control_points) - 1)).multiply(controlPointMatrix)
        self.initialize_degree_array()
        self.initialize_coefficient_array()

    # Initializes the degree/power array (for later processing) and cache them
    def initialize_degree_array(self):
        deg = len(self.control_points) - 1
        
        self.diff_powers = [[0 for _ in range(len(self.control_points))] for _ in range(3)]
        for i in range(len(self.diff_powers)):
            self.diff_powers[i] = self.gen_diff(deg, i)

    # Generate and return a polynomial's powers at the differentiation level
    def gen_diff(self, deg, diffLevel):
        output = [0 for _ in range(deg + 1)]

        i = diffLevel
        while i < len(output):
            output[i] = i - diffLevel
            i += 1

        return output

    # Initializes the coefficient array (for later processing) and cache them. Each row is a different level of differentiation.
    def initialize_coefficient_array(self):
        self.diff_coefficients = [[0 for _ in range(len(self.control_points))] for _ in range(3)]

        self.diff_coefficients[0] = [1 for _ in range(len(self.diff_coefficients[0]))]

        row = 1
        while row < len(self.diff_coefficients):
            col = 0
            while col < len(self.diff_coefficients[0]):
                self.diff_coefficients[row][col] = self.diff_coefficients[row-1][col] * self.diff_powers[row-1][col]
                col += 1
            row += 1

    # Returns the closest point t-value to the specified pose.
    def get_closest_point(self, pose, searchLimit, initialTValueGuess):
        searchEstimates = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, initialTValueGuess]
        closestDistance = 1e9
        bestGuess = 0

        for t in searchEstimates:
            dist = self.get_pose(t).distSquared(pose)

            if dist < closestDistance:
                closestDistance = dist
                bestGuess = t

        initialTValueGuess = bestGuess

        for i in range(searchLimit):
            pointAtT = self.get_point_characteristics(initialTValueGuess)
            lastPos = Pose(pointAtT.matrix[0][0], pointAtT.matrix[0][1])
            
            # Resultant matrix is:
            # [...] // first row is ignored
            # [(lastPos.x - pose.x) * x'(t) + (lastPos.y - pose.y) * y'(t)]
            # [(lastPos.x - pose.x) * x''(t) + (lastPos.y - pose.y) * y''(t)]

            resultant = pointAtT.multiply(Matrix(arr=[
                    [lastPos.getX() - pose.getX()], 
                    [lastPos.getY() - pose.getY()]
                ]))

            firstDerivative = 2 * resultant.matrix[1][0]
            # parentheses are messing me up rn, might have to recheck later
            secondDerivative = 2 * (resultant.matrix[2][0] + (pointAtT.matrix[1][0]**2 + pointAtT.matrix[1][1]**2))

            initialTValueGuess = utils.clamp(initialTValueGuess - firstDerivative / (secondDerivative + 1e-9), 0, 1)

            if self.get_pose(initialTValueGuess).distanceFrom(lastPos) < 0.1:
                break
        
        return initialTValueGuess

    def get_t_vector(self, t, diffLevel):
        degrees = self.diff_powers[diffLevel]
        powers = [0.0 for _ in range(len(self.control_points))]

        powers[0] = 1
        for i in range(1, len(powers)):
            powers[i] = t * powers[i - 1]
            
        output = [0.0 for _ in range(len(powers))]

        for i in range(len(degrees)):
            output[i] = powers[degrees[i]] * self.diff_coefficients[diffLevel][i]
        
        return output

    def get_pose(self, t):
        if t > 1:
            t = 1
        elif t < 0:
            t = 0
        
        outPos = Matrix(arr=[self.get_t_vector(t, 0)]).multiply(self.cached_matrix)
        return Pose(outPos.matrix[0][0], outPos.matrix[0][1])

    def get_point_characteristics(self, t):
        if t > 1:
            t = 1
        elif t < 0:
            t = 0

        return Matrix(arr=[self.get_t_vector(t, 0), self.get_t_vector(t, 1), self.get_t_vector(t, 2)]).multiply(self.cached_matrix)

    def get_derivative(self, t):
        if t > 1:
            t = 1
        elif t < 0:
            t = 0

        outVel = Matrix(arr=[self.get_t_vector(t, 1)]).multiply(self.cached_matrix)
        output = Vector()
        output.setOrthogonalComponents(outVel.matrix[0][0], outVel.matrix[0][1])
        return output
    
    def get_second_derivative(self, t):
        if t > 1:
            t = 1
        elif t < 0:
            t = 0

        outAccel = Matrix(arr=[self.get_t_vector(t, 2)]).multiply(self.cached_matrix)
        output = Vector()
        output.setOrthogonalComponents(outAccel.matrix[0][0], outAccel.matrix[0][1])
        return output

    def get_curvature(self, t):
        if t > 1:
            t = 1
        elif t < 0:
            t = 0

        derivative = self.get_derivative(t)
        second_derivative = self.get_second_derivative(t)
        if derivative.magnitude == 0:
            return 0
        
        return (derivative.cross(second_derivative)) / derivative.magnitude**3
    
    def get_normal_vector(self, t):
        tan = self.get_derivative(t).normalize()
        return Vector(magnitude=1.0, theta=tan.theta + math.pi/2)

    def at_parametric_end(self, t):
        return t >= 0.995

    # chat
    def get_path_completion(self, t):
        if t <= 0: 
            return 0.0
        if t >= 1:
            return 1.0

        # nearest among the sampled points in completion_map
        # (better: linear interpolate between neighbors)
        keys = sorted(self.completion_map.keys())
        # find neighbors
        i = bisect.bisect_left(keys, t)
        if i == 0:
            return self.completion_map[keys[0]] / self.length
        if i >= len(keys):
            return self.completion_map[keys[-1]] / self.length

        t0, t1 = keys[i-1], keys[i]
        s0, s1 = self.completion_map[t0], self.completion_map[t1]
        alpha = (t - t0) / (t1 - t0 + 1e-12)
        s = s0 + (s1 - s0) * alpha
        return s / self.length
    
class BezierLine(BezierCurve):
    def __init__(self, startPose, endPose):

        super().__init__()

        self.initialized = False
        self.length = None
        self.cached_matrix = Matrix()

        self.diff_powers = None
        self.diff_coefficients = None

        self.completion_map = dict()

        self.startPose = startPose
        self.endPose = endPose
        self.initialize()

        self.curvature = 0

    def initialize(self):
        if self.initialized:
            return
        
        self.initialized = True
        self.length = self.approximate_length()
        UNIT_TO_TIME = 1 / self.length
        endTangent = self.get_derivative(1).normalize()

    def get_derivative(self,t):
        returnVector = Vector()

        returnVector.setOrthogonalComponents(self.endPoint.getX() - self.startPoint.getX(), self.endPoint.getY() - self.startPoint.getY())

    def approximateLength(self):
        return math.sqrt((self.startPoint.getX() - self.endPoint.getX())**2 + (self.startPoint.getY() - self.endPoint.getY())**2)

    def get_closest_point(self, pose, searchLimit, initialTValueGuess):
        BA = Vector(self.endPose.minus(self.startPose))
        PA = Vector(pose.minus(self.startPose))

        return utils.clamp(BA.dot(PA) / (BA.magnitude)**2, 0, 1)

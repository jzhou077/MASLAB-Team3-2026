import time
import board
import busio
import math
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR

def heading_from_quat_deg(qw, qx, qy, qz):
    norm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
    if norm == 0:
        return None
    qw, qx, qy, qz = qw/norm, qx/norm, qy/norm, qz/norm
    siny_cosp = 2.0 * (qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    heading = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return heading % 360

class IMU:
    def __init__(self, start_heading=0):
        # User inputted initial heading (IN DEGREES)
        self.start_heading = start_heading

        self.i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)  # start at 400k
        self.bno = BNO08X_I2C(self.i2c)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        time.sleep(0.5)

        # Initial raw heading (based on magnetic poles)
        self.initial_raw_heading = self.get_heading_deg()
        print("IMU initialized. Initial heading:", self.find_adjusted_heading())

    def get_heading_deg(self):
        quat = self.bno.quaternion
        if quat is None:
            return None
        qi, qj, qk, qr = quat
        if qi == 0 and qj == 0 and qk == 0 and qr == 0:
            return None
        return heading_from_quat_deg(qr, qi, qj, qk)

    def find_adjusted_heading(self):
        current_heading = self.get_heading_deg()
        if current_heading is None or self.initial_raw_heading is None:
            return None
        delta_heading = current_heading - self.initial_raw_heading
        adjusted_heading = (self.start_heading + delta_heading) % 360
        if adjusted_heading < 0:
            return adjusted_heading + 360
        
        return adjusted_heading
    
    def reset(self):
        self.initial_raw_heading = self.get_heading_deg()
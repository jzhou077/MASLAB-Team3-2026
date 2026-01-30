import time

class Timer:
    def __init__(self):
        self.start_time = time.time()

    def reset(self):
        self.start_time = time.time()

    def getElapsedTime(self):
        return time.time() - self.start_time
    
class NanoTimer:
    def __init__(self):
        self.start_time = time.time_ns()

    def reset(self):
        self.start_time = time.time_ns()

    def getElapsedTime(self):
        return time.time_ns - self.start_time
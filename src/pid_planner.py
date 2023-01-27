import time
from frenet import distance

class PI:
    def __init__(self, P = 0.90, I = 100000, current_time = None):
        self.Kp = P
        self.Ki = I

        self.sample_time = 0.1
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        self.SetPoint = [0.0, 0.0]
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.last_error = 0.0

        self.output = 0.0

    def update(self, feedback_value, current_time=None):
        error =  distance(feedback_value[0], feedback_value[1], self.SetPoint[0], self.SetPoint[1])
        
        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if delta_time <= self.sample_time:
            self.PTerm = self.Kp*delta_error
            self.ITerm += error*delta_time

            self.last_time = self.current_time
            self.last_error = error

        self.output = self.PTerm + self.Ki * self.ITerm 
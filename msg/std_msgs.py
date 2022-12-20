import time

class Header:
    def __init__(self, seq=None, stamp=None, frame_id=None):
        self.seq = seq                                           # uint32
        self.stamp = time.time()                                       # time stamp
        self.frame_id = frame_id 
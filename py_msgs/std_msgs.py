import time

# ros std_msgs dataclasses

class Header:
    def __init__(self, seq=None, stamp=None, frame_id=None):
        self.seq = seq                                           # uint32
        stamp = time.time()
        self.stamp = stamp                                       # time stamp
        self.frame_id = "/base_link" 
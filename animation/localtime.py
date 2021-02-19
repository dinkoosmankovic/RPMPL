from timeit import default_timer as timer
import time


class LocalTime:
    def __init__(self):
        self.start = timer()

    def tick(self, ticks):
        end = timer()
        # ticks in miliseconds
        ticks /= 1000
        diff = end - self.start
        if diff < ticks:
            time.sleep(ticks - diff)
        self.start = timer()


import sys
import time


# class to print out progress
class Progress:
    def __init__(self, bagsize):
        self.bagsize = bagsize
        self.ncount = 0
        self.last_time = time.monotonic()

    def myfunc(self):
        print("Hello my name is " + self.name)

    def status(self, length, percent):
        sys.stdout.write('\x1B[2K') # Erase entire current line
        sys.stdout.write('\x1B[0E') # Move to the beginning of the current line
        progress = "Progress: ["
        for i in range(0, length):
            if i < length * percent:
                progress += '='
            else:
                progress += ' '
        progress += "] " + str(round(percent * 100.0, 2)) + "%"
        sys.stdout.write(progress)
        sys.stdout.flush()

    def update_progress(self):
        #progress
        self.ncount += 1
        if time.monotonic() - self.last_time > .1:
            percent = self.ncount / self.bagsize
            self.status(40, percent)
            self.last_time = time.monotonic()

    def progress_done(self):
        self.status(40, 1)
        print("\n")

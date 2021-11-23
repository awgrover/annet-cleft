'''
# Every
#
# Periodic actions, and "one-shot" timers
# and patterns of those.

from unrvl.every import Every

# setup periodics, patterns, timers
every_half_second = Every(0.5) # every 0.5 seconds
pattern1 = Every(1,0.1) # 1 second, then 0.1 second. pattern
in_two_seconds = Every(2, 0) # does not repeat, note the 0
tap_sequence = Every(1.0, 0.1, 1, 0.1, 0) # does not repeat, a pattern

# other setup
cp.detect_taps = 1
Off = (0,0,0)
tap_color = (0,10,30)

while (1):
    # on/off periods = 0.5 seconds.
    if every_half_second():
        cp.red_led = not cp.red_led # blink

    # on is 1 second, off is 0.1
    if pattern1():
        if pattern1.i==0:
            cp.pixels[0] = (30,0,10)
        else:
            cp.pixels[0] = Off

    # when something happens, turn on an led for 1 second
    if cp.tapped:
        cp.pixels[1] = tap_color
        # timers don't run till explicitly started
        tap_sequence.start # start it
    # when each step of that timer expires...
    if tap_sequence(): 
        # .i is the _next_ step: 1,2,3,..
        if tap_sequence.i % 2: # 1,0,1...
            cp.pixels[1] = Off
        else
            cp.pixels[1] = tap_color

'''

__version__ = "1.0"

import time

class Every(object):
    def __init__(self, *interval):
        # Make an instance.
        #   :interval in seconds

        self.running = True # modified by .interval=
        self.interval = interval
        # we pretend to start at last, for the immediate-expire case
        self.i = len(self.interval)-1
        self.last = time.monotonic() - interval[self.i] # start immediatly

    @property
    def interval(self):
        return self.__interval

    @interval.setter
    def interval(self,v):
        '''tolerate single value or tuple-pattern'''
        if isinstance(v,tuple):
            self.__interval = v
        elif isinstance(v, int) or isinstance(v, float):
            self.__interval = (v,) # allways tuples
        else:
            raise Exception(".interval must be a number or tuple")
        self.i=0
        self.last = time.monotonic() - self.interval[self.i] # start immediatly
        # timers (final 0) don't run till .start
        self.running = self.interval[-1] != 0
        return self

    def start(self):
        self.last = time.monotonic()
        self.running = True
        self.i=0
        return self

    def __call__(self):
        # true when the current interval expires
        now = time.monotonic()
        diff = now - self.last

        this_interval = self.interval[self.i]
        if (self.running and this_interval != 0 and diff >= this_interval):
            self.last = now
            last_interval = self.interval[self.i]
            self.i = (self.i + 1) % len(self.interval)
            next_interval = self.interval[self.i]
            if next_interval != 0:
                drift = diff % last_interval
                self.last -= drift
            else:
                self.running = False
            return True
        else:
            return False

class Timer(Every):
    # convenience for Every(a,b,0), i.e. one-shot
    def __init__(self, *interval):
        # add the ,0
        super().__init__( *( tuple(list(interval) + [0]) ))

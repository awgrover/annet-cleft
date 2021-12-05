import sys
from abc import ABC, abstractmethod
from exponential_smooth import ExponentialSmooth
from copy import copy
from ring import Ring

class AnalyzerInterface:
    """Implement this in analyzers that are Analyze.add( some_analyzer )"""
    @abstractmethod
    def __call__(self,x,y,temp):
        pass

    @abstractmethod
    def next(self):
        pass

    @abstractmethod
    def print_stats(self):
        pass

    @abstractmethod
    def post(self):
        """do the whole frame analysis"""
        pass

class Analyze:
    # Analyze data as it comes in: analyzer(x,y,temp)
    # Analyze the whole frame of data: analyzer()
    # .print_stats() to print it
    # Reset for next frame analyzer.next()
    #
    # Add .accumulator( new someAnalyzerInterface )
    # add .post( new somePostInterface )

    def __init__(self):
        self.accumulators = []
        self.posts = []
        self.next()

    def accumulator(self,analyzer):
        """Add an analyzer that accumulates each temperature
        runs at .analyze(x,y,tmp) time"""
        if not isinstance(analyzer, AnalyzerInterface):
            raise TypeError("{} is not an AnalyzerInterface".format(analyzer.__class__.__name__))
        self.accumulators.append( analyzer )

    def post(self,analyzer):
        """Add an analyzer that runs at .analyze() time"""
        if not isinstance(analyzer, AnalyzerInterface):
            raise TypeError("{} is not an AnalyzerInterface".format(analyzer.__class__.__name__))
        self.posts.append( analyzer )

    def next(self):
        self.ct = 0
        for a in self.accumulators:
            a.next()
        for a in self.posts:
            a.next()

    def print_stats(self):
        print("  pixels {}".format(self.ct))
        for a in self.accumulators:
            print("  from {}".format(a.__class__.__name__))
            a.print_stats()
        for a in self.posts:
            print("  from {}".format(a.__class__.__name__))
            a.print_stats()

    def __call__(self, x=-1, y=-1, temp=0):
        """To add a value for analysis (x,y,temp),
        or to signal end-of-frame ()
        """

        if (x == -1):
            for a in self.posts:
                a.post()
        else:
            self.ct += 1
            for a in self.accumulators:
                a(x,y,temp)

from map_range import map_range

class Histo(AnalyzerInterface):
    """Keep a histogram of values, from low..high
        methods to convert from index to values
    """
    def __init__(self, bucket_ct, low_v, high_v):
        self.bucket_ct = bucket_ct
        self.low_value = low_v
        self.high_value = high_v
        self.bucket_width = (high_v - low_v) / (1.0 * bucket_ct)
        self.next()

    def next(self):
        self.buckets = [0 for _ in range(self.bucket_ct)]

    def __call__(self, x,y, temp):
        if temp < self.low_value:
            temp = self.low_value
        if temp > self.high_value:
            temp = self.high_value
        i = map_range( temp, self.low_value, self.high_value, 0, self.bucket_ct-1)
        self.buckets[int(i)] += 1

    def post():
        pass

    def value_at(self, i):
        return map_range( i, 0, self.bucket_ct, self.low_value, self.high_value) + self.bucket_width/2

    def i_at(self, value): 
        # the [i] of value in buckets
      return int(map_range( value, self.low_value, self.high_value, 0, self.bucket_ct-1))

    def print_stats(self):
        #print("  histo")
        #print("    buckets {}, low/high {:0.2f}/{:0.2f} width {:0.2f}"
        #    .format( self.bucket_ct, self.low_value, self.high_value, self.bucket_width )
        #    )

        sys.stdout.write("histo[") # PROCESSING
        sys.stdout.write(str(self.bucket_ct))
        sys.stdout.write(",")
        for i,v in enumerate(self.buckets):
            if not (i % 10):
                sys.stdout.write(" ")
            sys.stdout.write(str(v))
            sys.stdout.write(",")
        print("]")

        sys.stdout.write("histotemps[") # PROCESSING
        sys.stdout.write(str(self.bucket_ct))
        sys.stdout.write(",")
        for i in range(self.bucket_ct):
            if not (i % 10):
                sys.stdout.write(" ")
            sys.stdout.write("{:0.2f}".format( self.value_at(i) ))
            sys.stdout.write(",")
        print("]")

class TemperatureLocation:
    """holds a temp, i, x,y"""
    def __init__(self,temp, i, x, y):
        self.temp = temp
        self.i = i
        self.x = x
        self.y = y

    def to_print(self):
        # i,temp,x,y
        return "{},{:0.2f},{},{}".format( 
            self.i,self.temp, 
            self.x,
            self.y
            )

class MinMax(AnalyzerInterface):
    """Min & max"""
    def __init__(self, histo):
        self.histo = histo
        self.next()

    def next(self):
        self.low = TemperatureLocation(1000.0, -1, -1, -1)
        self.high = TemperatureLocation(0.0, -1, -1, -1)

    def __call__(self,x,y,temp):
        if (self.low.temp > temp):
            self.low.temp = temp
            self.low.i = self.histo.i_at( self.low.temp )
            self.low.x=x
            self.low.y=y
        if (self.high.temp < temp):
            self.high.temp = temp
            self.high.i = self.histo.i_at( self.high.temp )
            self.high.x=x
            self.high.y=y

    def post():
        # not used
        pass

    def print_stats(self):
        # mintemp[temp,i,x,y,]
        # maxtemp[[temp,i,x,y,]
        print("mintemp[{},]".format( self.low.to_print() ))
        print("maxtemp[{},]".format( self.high.to_print() ))

class FirstHigh(AnalyzerInterface):
    """Looks in the histo for the pattern:
    ...XXXX..YY
    And calls the first Y "first high"
    """

    def __init__(self, histo):
        self.histo = histo
        self.next()

    def next(self):
        self.temp = 0.0

    def __call__(self,x,y,temp):
        pass

    def post(self):
        #print("FirstHigh")
        last_low_temp_i = self.find_at_least_in_a_row_over_fence(0, 4, 1, over=True) # starting, at_least_in_a_row, fence
        if (last_low_temp_i == 0):
            self.temp_i = 0
            return

        last_gap_temp_i = self.find_at_least_in_a_row_over_fence(last_low_temp_i, 2, 1, over=False)
        if (last_gap_temp_i == 0):
            self.temp_i = 0
            return

        self.temp_i = last_gap_temp_i + 1
        self.temp = self.histo.value_at(self.temp_i)

    def print_stats(self):
        print("firsthigh[{},{:0.2f},]".format(self.temp_i, self.temp ))

    def find_at_least_in_a_row_over_fence(self, starting_i, at_least_in_a_row, fence, over=True):
      # find at_least_in_a_row buckets in a row that are ct >= fence
      # (reversed means <= fence)

      #print("find fenced at_least_in_a_row {} {} starting [{}]".format(
      #  at_least_in_a_row, (">" if over else "<"), fence, starting_i
      #  ))

      fenced_i = starting_i
      fenced_ct = 0

      while (fenced_ct < at_least_in_a_row): # restart first group until ct found

        # find next temp w/counts above fence
        found = False
        while ( fenced_i < self.histo.bucket_ct - at_least_in_a_row ):
            over_fence = (self.histo.buckets[fenced_i] > fence) if over else (self.histo.buckets[fenced_i] < fence) 
            if (over_fence):
                found = True
                #print("Min start {}/{:0.2f}".format(fenced_i, self.histo.value_at(fenced_i) ))
                break
            fenced_i += 1

        if (not found):
          #print("no 0 buckets found by {}".format(fenced_i))
          return 0

        # find at_least_in_a_row below-fence in a row
        fenced_ct = 0
        for i in range(fenced_i, self.histo.bucket_ct - at_least_in_a_row):
            if i >= fenced_i + at_least_in_a_row:
                break
            is_over_fence = (self.histo.buckets[i] <= fence) if over else (self.histo.buckets[i] >= fence)
            if (is_over_fence):
                break
            fenced_ct += 1 # gauranteed fence_ct at least 1

        if (fenced_ct < at_least_in_a_row):
          #print("Not at_least_in_a_row in a row {} at [{}]".format(at_least_in_a_row, (fenced_i + fenced_ct - 1) ))
          # so, will retry till we run out of bins
          fenced_i += fenced_ct # go past for next

      # we have at_least_in_a_row in a row (in fact, fenced_ct in a row)

      #print("Fenced start [{}] .. {}".format(fenced_i, (fenced_ct + fenced_i - 1) ))
      return (fenced_ct + fenced_i - 1) # last value

class BackgroundTemp(AnalyzerInterface):
    """Post for tracking background temp"""
    def __init__(self, minmax, histo, firsthigh):
        self.minmax = minmax
        self.histo = histo
        self.firsthigh = firsthigh
        # our comparison with firsthigh keeps us below it despite tracking:
        self.background_temp = ExponentialSmooth(100.0) # slow track
        self.next()

    def __call__(self,x,y,temp):
        pass

    def next(self):
        # we never reset
        pass

    def print_stats(self):
        temp = self.background_temp.value
        print( "background[{},{:0.2f},]".format( self.histo.i_at( temp ), temp ))

    def post(self):
        """do the whole frame analysis"""

        # ignore early values
        if self.minmax.high.temp > 18.0:

            # start with first minmax.high
            if self.background_temp.value == 0:
                if self.firsthigh.temp > 18.0:
                    self.background_temp.reset( self.firsthigh.temp )
                else:
                    self.background_temp.reset( self.minmax.high.temp )
            
            # quickly backoff if the max gets colder == background
            elif self.minmax.high.temp > 10 and self.background_temp.value > self.minmax.high.temp:
                self.background_temp.reset( self.minmax.high.temp )

            # track the max, assuming it is background
            elif self.minmax.high.temp > 10: # ignore glitches
                # we want to stay below firsthigh
                # if we are 3 bins below it, or 3 above it
                to_heat_island = self.firsthigh.temp - self.background_temp.value
                #print("ISL @{} fh{} {} +-{}".format(self.background_temp.value, self.firsthigh.temp, to_heat_island, 3*self.histo.bucket_width))
                if to_heat_island >= (-3*self.histo.bucket_width) and to_heat_island <= (3*self.histo.bucket_width): 
                    print("  reset to {}".format(self.firsthigh.temp - 2*0.25))
                    self.background_temp.reset( self.firsthigh.temp - 2*0.25)
                else:
                    # track the max
                    self.background_temp.update( self.minmax.high.temp );

class XY:
    def __init__(self, x, y):
        self.x=x
        self.y=y

    def __str__(self):
        return "({},{})".format(self.x,self.y)

    def __eq__(self, other):
        if isinstance(other, XY):
            return self.x==other.x and self.y==other.y
        elif isinstance(other, list) or isinstance(other, tuple):
            return self.x==other[0] and self.y==other[1]
        else:
           raise TypeException("don't know how to == vs a {}".format(type(other)))

class HotSpot(AnalyzerInterface):
    """Find hotspot if > background, classify it, and movement"""

    def __init__(self, minmax, background):
        self.minmax = minmax
        self.background = background
        self.was_seen = False

        self.hotspot = TemperatureLocation(0,-1,-1,-1) # we'll track this over time
        self.was = XY(-1,-1)
        self.now = XY(-1,-1)
        self.next()

    def __call__(self,x,y,temp):
        pass

    def next(self):
        pass

    def print_stats(self):
        # x,y, quadrant
        # quadrant is really 9'ant, i.e. 3x3:
        #         nearest/front
        #        0,0 | 0,1 | 0,2
        #   left 1,0 | 1,1 | 1,2 right
        #        2,0 | 2,1 | 2,2
        #           farthest
        # plus "empty" [-1,-1]
        # Our state is: was -> is, until we are told "seen" (then was=is)
        print( "hotspot[{},{}, {},{},]".format(self.was.x, self.was.y, self.now.x, self.now.y) )

    def post(self):
        """do the whole frame analysis"""
        # ignore glitches
        if self.background.firsthigh.temp > 10:

            # if we have the nice pattern (detected the "gap"):
            # background < firsthigh < max
            # then use max
            use_temp = None
            if (
                self.background.background_temp.value < self.background.firsthigh.temp
                and self.background.firsthigh.temp <= self.minmax.high.temp
                ):
                use_temp = self.minmax.high

            # Less certain is when we don't have a good "gap"
            # in fact, probably not useful since background-tracking
            # tends to be a bit less than max all the time...
            elif (
                self.background.background_temp.value < self.minmax.high.temp
                and (self.minmax.high.temp - self.background.background_temp.value) > 0.5
                and self.background.firsthigh.temp >= self.minmax.high.temp
                ):
                # use max, but maybe we want to realize that this is less certain...
                use_temp = self.minmax.high

            if use_temp:
                #if not boredwithspot
                self.hotspot = use_temp


                # update was/now if the last was seen
                if self.was_seen:
                    self.was_seen = False
                    self.was = copy(self.now)

                    # hystersis by doing 2 border 2 border 2 and counting border as the last quadrant

                    self.now.x = self._quadrant( self.hotspot.x )
                    self.now.y = self._quadrant( self.hotspot.y )
                    #print("animation now ({},{}) from hotat [{},{}]".format(self.now.x,self.now.y, self.hotspot.x,self.hotspot.y))

            else:
                # so, must not be anything in view
                if self.was_seen:
                    self.now.x = -1
                    self.now.y = -1

    def seen(self):
        self.was_seen = True

    def _quadrant(self, xy ):
        # x & y are symmetrical, so same decision

        # 3,2,3 so center is narrow
        if xy <= 2:
            return 0
        elif xy <= 4:
            return 1
        else:
            return 2

# see arduino/cleft/commands.h and cleft.ino Animation* Animation::animations[] =...
# i.e. you'll need an animation in that array to correspond
IDLE = 'a'
RELAX = chr(ord(IDLE)+1)
REAR_UP = chr(ord(RELAX)+1)
JITTER = chr(ord(REAR_UP)+1)
WORN_POSTURE = chr(ord(JITTER)+1)
LEFT_WAVE = chr(ord(WORN_POSTURE)+1)
RIGHT_WAVE = chr(ord(LEFT_WAVE)+1)

choose_animation_raw_history = Ring( ord(RIGHT_WAVE) - ord(IDLE) + 1, IDLE ) # the "raw" 

def choose_animation( hotspot ):
    # return an animation #

    # if we don't choose/send an aniimation, the arduino should assume no-effect
    choice = None

    # The instability means we need more rules that are sort of like debounce
    # So, us last_choices as the history and decide on that

    # no-one in view
    if hotspot.was == (-1,-1) and hotspot.now == (-1,-1):
        hotspot.seen()
        #print("animation case IDLE because was & now -1")
        # inside -> exit => RELAX, None, IDLE # takes 3 steps
        choice = if_ntimes(2, IDLE)

    # just exited view
    elif hotspot.now == (-1,-1):
        hotspot.seen()
        #print("animation case RELAX because exited now -1")
        choice = if_ntimes(2, RELAX)

    # approached to edge (back)
    elif hotspot.was == (-1,-1) and hotspot.now.y == 2:
        hotspot.seen()
        #print("animation case REAR_UP because was -1, now.y==2")
        # the RELAX and IDLE "debounce" should make this not need debounce
        choice = if_ntimes(1, REAR_UP) # immediate, exiting is debounced

    # l|r|back -> center : jitter
    elif hotspot.was != (1,1) and hotspot.was.y != 0 and hotspot.now == (1,1):
        hotspot.seen()
        #print("animation case JITTTER because was not center, now 1,1")
        choice = if_ntimes(1,JITTER) # immediate, can't oscillate with worn

    # center-front: worn posture
    elif hotspot.now == (1,0):
        hotspot.seen()
        #print("animation case WORN because front 1,0")
        # fast start jitter, debounce the others that are "exiting" worn
        choice = if_ntimes(1,WORN_POSTURE) # with breathing, immediate, adjacents are debounced

    # left/right : wave big, then small slow (caress)
    elif hotspot.now.x == 0 or hotspot.now.x == 2:
        hotspot.seen()
        might_be = LEFT_WAVE if hotspot.now.x == 0 else RIGHT_WAVE
        if hotspot.was == (-1,-1):
            #print("animation case LEFT because x=0, and from outside")
            choice = might_be # immediate on "entry"
        else:
            # debounce
            #print("animation case LEFT because x=0, but debounce")
            choice = if_ntimes(2, (LEFT_WAVE if hotspot.now.x == 0 else RIGHT_WAVE) )

    else:
        #print( "animation NOT CATEGORIZED {},{}, {},{}".format(hotspot.was.x, hotspot.was.y, hotspot.now.x, hotspot.now.y) )
        hotspot.seen()
        choice = if_ntimes(2, None) # covers worn->jitter debounce
        choice = None

    return choice if choice else None

def if_ntimes(n, animation):
    """does animation appear n times in a row from now, including this time: return animation else False"""
    choose_animation_raw_history.insert(animation)
    for counting in range(n):
        if choose_animation_raw_history[counting] != animation:
            return False
    return animation

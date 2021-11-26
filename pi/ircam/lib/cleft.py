import sys
from abc import ABC, abstractmethod
from exponential_smooth import ExponentialSmooth

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
          print("no 0 buckets found by {}".format(fenced_i))
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
            else:
                # we want to stay below firsthigh
                # if we are 3 bins below it, or 3 above it
                to_heat_island = self.firsthigh.temp - self.background_temp.value
                #print("ISL @{} fh{} {} +-{}".format(self.background_temp.value, self.firsthigh.temp, to_heat_island, 3*self.histo.bucket_width))
                if to_heat_island >= (-3*self.histo.bucket_width) and to_heat_island <= (3*self.histo.bucket_width): 
                    #print("  reset to {}".format(self.firsthigh.temp - 2*self.histo.bucket_width))
                    self.background_temp.reset( self.firsthigh.temp - 2*self.histo.bucket_width )
                else:
                    # track the max
                    self.background_temp.update( self.minmax.high.temp );

class XY:
    def __init__(self, x, y):
        self.x=x
        self.y=y

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
        self.was_seen = True

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
        print( "hotspot[{},{}, {},{}]".format(self.was.x, self.was.y, self.now.x, self.now.y) )

    def post(self):
        """do the whole frame analysis"""
        if self.background.firsthigh.temp > 0 and self.minmax.high.temp > self.background.firsthigh.temp:
            #if not boredwithspot
            self.hotspot = self.minmax.high


            # update was/now if the last was seen
            if self.was_seen:
                self.was_seen = False
                self.was = self.now

                # hystersis by doing 2 border 2 border 2 and counting border as the last quadrant

                self.now.x = self._quadrant( self.hotspot.x )
                self.now.y = self._quadrant( self.hotspot.y )
                print("now ({},{}) from hotat [{},{}]".format(self.now.x,self.now.y, self.hotspot.x,self.hotspot.y))

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

IDLE = 3
RELAX = IDLE+1
REAR_UP = RELAX+1
JITTER = REAR_UP+1
WORN_POSTURE = JITTER+1
LEFT_WAVE = WORN_POSTURE+1
RIGHT_WAVE = LEFT_WAVE+1

def choose_animation( hotspot ):
    # return an animation #

    # if we don't choose/send an aniimation, the arduino should assume IDLE

    # still no-one
    if hotspot.was == (-1,-1) and hotspot.now == (-1,-1):
        return IDLE

    # just left
    elif hotspot.now == (-1,-1):
        hotspot.seen()
        return RELAX

    # approached to edge (front)
    elif hotspot.was == (-1,-1) and hotspot.now.y == 2:
        hotspot.seen()
        return REAR_UP

    # l|r|back -> center : jitter
    elif hotspot.was != (1,1) and hotspot.was.y != 0 and hotspot.now == (1,1):
        hotspot.seen()
        return JITTER

    # center-front: worn posture
    elif hotspot.now == (0,1):
        hotspot.seen()
        return WORN_POSTURE # with breathing

    # left/right : wave big, then small slow (caress)
    elif hotspot.now.x == 1:
        hotspot.seen()
        return LEFT_WAVE

    # left/right : wave big, then small slow (caress)
    elif hotspot.now.x == 2:
        hotspot.seen()
        return RIGHT_WAVE

    else:
        print("NOT CATEGORIZED")
        return None

import sys
from abc import ABC, abstractmethod

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
    def post():
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
        """Add an analyzer that accumulates each temperature"""
        if not isinstance(analyzer, AnalyzerInterface):
            raise TypeError("{} is not an AnalyzerInterface".format(analyzer.__class__.__name__))
        self.accumulators.append( analyzer )

    def post(self,analyzer):
        """Add an analyzer that runs at analyze() time"""
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

    def print_stats(self):
        print("  histo")
        print("    buckets {}, low/high {:0.2f}/{:0.2f} width {:0.2f}"
            .format( self.bucket_ct, self.low_value, self.high_value, self.bucket_width )
            )

        sys.stdout.write("histo[") # PROCESSING
        sys.stdout.write(str(self.bucket_ct))
        sys.stdout.write(",")
        for v in self.buckets:
            sys.stdout.write(str(v))
            sys.stdout.write(",")
        print("]")

        sys.stdout.write("histotemps[") # PROCESSING
        sys.stdout.write(str(self.bucket_ct))
        sys.stdout.write(",")
        for i in range(self.bucket_ct):
            sys.stdout.write("{:0.2f}".format( self.value_at(i) ))
            sys.stdout.write(",")
        print("]")

class MinMax(AnalyzerInterface):
    """Min & max"""
    def __init__(self):
        self.next()

    def next(self):
        self.low = 1000.0
        self.high = 0.0

    def __call__(self,x,y,temp):
        if (self.low > temp):
            self.low = temp
        if (self.high < temp):
            self.high = temp

    def post():
        pass

    def print_stats(self):
        print("minmax[{:f},{:f},]".format(self.low,self.high)) # PROCESSING

class FirstHigh(AnalyzerInterface):
    """Looks in the histo for the pattern:
    ...XXXX..YY
    And calls the first Y "first high"

    def __init__(self):
        self.next()

    def next(self):
        self.low = 1000.0
        self.high = 0.0

    def __call__(self,x,y,temp):
        if (self.low > temp):
            self.low = temp
        if (self.high < temp):
            self.high = temp

    def post():
        pass

    def print_stats(self):
        print("minmax[{:f},{:f},]".format(self.low,self.high)) # PROCESSING


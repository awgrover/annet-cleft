import sys
from abc import ABC, abstractmethod

class AnalyzerInterface:
    """Implement this in analyzers that are Analyze.add( some_analyzer )"""
    @abstractmethod
    def __call__(x,y,temp):
        pass

    @abstractmethod
    def print():
        pass

class Analyze:
    # holds a bunch of analyzers
    def __init__(self):
        self.analyzers = []
        self.next()

    def add(self,analyzer):
        if not isinstance(analyzer, AnalyzerInterface):
            raise TypeError("{} is not an AnalyzerInterface".format(analyzer.__class__.__name__))
        self.analyzers.append( analyzer )

    def next(self):
        self.ct = 0
        for a in self.analyzers:
            a.next()

    def print_stats(self):
        print("  pixels {}".format(self.ct))
        for a in self.analyzers:
            print("  from {}".format(a.__class__.__name__))
            a.print_stats()

    def __call__(self, x, y, temp):
        """To add a value for analysis"""
        self.ct += 1
        for a in self.analyzers:
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

    def print_stats(self):
        print("  histo")
        print("    buckets {}, low/high {:0.2f}/{:0.2f} width {:0.2f}"
            .format( self.bucket_ct, self.low_value, self.high_value, self.bucket_width )
            )
        sys.stdout.write("histo[")
        sys.stdout.write(str(self.bucket_ct))
        sys.stdout.write(",")
        for v in self.buckets:
            sys.stdout.write(str(v))
            sys.stdout.write(",")
        print("]")



class Ring:
    """Simplified lifo, with [], i.e. "last n values"
    Does not block insertion, just overwrites.
    init fill with some value,
    then overwrites with append()
    [] is relative to the current read position
    """

    """
    head is where to read from
    append inserts at tail
    so [i] is [head % count]
    append is [head] head++
    """

    def __init__(self, count, init_value):
        """fill with init_value"""
        self._ring = [init_value] * count
        self.head = 0
        self.count = count

    def __str__(self):
        items = [ '{!r}'.format( item ) for item in self ]
        return '[' + ', '.join(items) + ']'

    def __getitem__(self, i):
        # [i] relative to head, so lifo
        return self._ring[ (self.head + i) % self.count ]

    def append(self, v):
        """insert at head, overwriting, increment, so that is last now"""
        self._ring[self.head] = v
        self.head = (self.head + 1) % self.count
        return self
    
    @property
    def ring(self):
        """as a list"""
        return [item for item in self]

    # generator
    def __iter__(self):
        """we support "for x in x"""
        return self.Iterator(self)
    
    class Iterator:
        def __init__(self, ring):
            self.the_ring = ring
            self.i = 0

        def __next__(self):
            if self.i == self.the_ring.count:
                raise StopIteration()
            else:
                rez = self.the_ring[self.i]
                self.i += 1
                return rez

#!/usr/bin/env python3
import sys
sys.path.append("./lib")
from ring import Ring

# A "ring" should let me see the n most recent values, such that most recent is [0]
# that's a fifo
# but, we don't block on insert, we overwrite

fring = Ring(5,1)
assert str(fring) == "[1, 1, 1, 1, 1]", "inited value {}".format(fring)


fring.insert( 2 )
assert str(fring) == "[2, 1, 1, 1, 1]", "append 2 value {}".format(fring)
assert str(fring.ring) == "[2, 1, 1, 1, 1]", ".ring same, saw {}".format( str(fring.ring ) )

fring.insert( 3 )
assert str(fring) == "[3, 2, 1, 1, 1]", "append 3 value {}".format(fring)

assert fring[0] == 3, "saw {}".format(fring)
assert fring[1] == 2, "saw {}".format(fring)
assert fring[2] == 1, "saw {}".format(fring)

assert fring.insert(4) == fring, "returns self, saw {}".format(fring.__class__.__name__)
assert str(fring) == "[4, 3, 2, 1, 1]", "append 4 value {}".format(fring)

assert fring.insert(5)
assert str(fring) == "[5, 4, 3, 2, 1]", "append 5 value {}".format(fring)

assert fring.insert(6)
assert str(fring) == "[6, 5, 4, 3, 2]", "append 6 (overwrite) value {}".format(fring)

assert fring[-1] == 2, "saw {}".format(fring[-1])
assert fring[-2] == 3, "saw {}".format(fring[-2])
assert fring[-5] == 6, "[-count-1] should be first saw {}".format(fring[-5])
assert fring[ fring.count ] == 6, "[count] should be first saw {}".format(fring[-5])
assert fring[ fring.count + 1] == 5, "wrap around saw {}".format(fring[-5])

print("passed")




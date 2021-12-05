#!/usr/bin/env python3
import sys
sys.path.append("./lib")
from ring import Ring

fring = Ring(5,1)
assert str(fring) == "[1, 1, 1, 1, 1]", "inited value {}".format(fring)


fring.append( 2 )
assert str(fring) == "[1, 1, 1, 1, 2]", "append 2 value {}".format(fring)
assert str(fring.ring) == "[1, 1, 1, 1, 2]", ".ring same, saw {}".format( str(fring.ring ) )

fring.append( 3 )
assert str(fring) == "[1, 1, 1, 2, 3]", "append 3 value {}".format(fring)

assert fring[0] == 1, "saw {}".format(fring[0])
assert fring[3] == 2, "saw {}".format(fring[3])
assert fring[4] == 3, "saw {}".format(fring[4])

assert fring.append(4) == fring, "returns self, saw {}".format(fring.__class__.__name__)
assert str(fring) == "[1, 1, 2, 3, 4]", "append 4 value {}".format(fring)

assert fring.append(5)
assert str(fring) == "[1, 2, 3, 4, 5]", "append 5 value {}".format(fring)

assert fring.append(6)
assert str(fring) == "[2, 3, 4, 5, 6]", "append 6 (overwrite) value {}".format(fring)

assert fring[-1] == 6, "saw {}".format(fring[-1])
assert fring[-2] == 5, "saw {}".format(fring[-2])
assert fring[-5] == 2, "[-count-1] should be first saw {}".format(fring[-5])
assert fring[ fring.count ] == 2, "[count] should be first saw {}".format(fring[-5])
assert fring[ fring.count + 1] == 3, "wrap around saw {}".format(fring[-5])

print("passed")




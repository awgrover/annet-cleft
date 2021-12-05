#!/usr/bin/env python3
import sys
sys.path.append("./lib")
import cleft
from copy import copy
from ring import Ring

def test_if_ntimes():
    assert cleft.if_ntimes(1, cleft.IDLE) == cleft.IDLE, "because it inserts first: {}".format(cleft.choose_animation_raw_history)
    assert cleft.if_ntimes(2, cleft.IDLE) == cleft.IDLE, "because twice: {}".format(cleft.choose_animation_raw_history)
    assert cleft.if_ntimes(2, cleft.RELAX) == False, "because only once: {}".format(cleft.choose_animation_raw_history)

    print("if_ntimes passed")

class TestHotspot:
    # mock for the needed hotspot object
    zones = [
        (0,0), (1,0), (2,0), # front, across 0,1,2
        (0,1), (1,1), (2,1), # mid, across 3,4,5
        (0,2), (1,2), (2,2), # back, across 6,7,8
        (-1,-1) # outside [-1]
        ]
    def __init__(self, from_zone, to_zone):
        self.now = cleft.XY(*self.zones[to_zone])
        #print(" TH.now {} == {} -> {}".format(to_zone, self.zones[to_zone], self.now))
        self.was = cleft.XY(*self.zones[from_zone])
        #print(" TH.was {} == {} -> {}".format(from_zone, self.zones[to_zone], self.was))

    def seen(self):
        pass

def test_choose_animation():

    # "outside" is just idle
    # and initially so
    rez = cleft.choose_animation( TestHotspot( -1, -1) ) 
    assert rez == cleft.IDLE,"saw {} history {}".format(rez, cleft.choose_animation_raw_history)

    # takes 3 "outside" to go to idle
    history_reset( cleft.RELAX )
    _test_choose( TestHotspot( 4, -1) , cleft.RELAX )
    _test_choose( TestHotspot( -1, -1) , None) # no change yet
    _test_choose( TestHotspot( -1, -1) , cleft.IDLE) # took 2

    # jitter start
    history_reset( )
    _test_choose( TestHotspot( -1, 4 ), cleft.JITTER) # immediate
    _test_choose( TestHotspot( 6, 4 ), cleft.JITTER) # immediate
    _test_choose( TestHotspot( 4, 4 ), None) # none
    # back to center, from front is NOT jitter
    _test_choose( TestHotspot( 2, 4 ), None) # nope!

    # left/right wave
    _test_choose( TestHotspot( 4, 5 ), None) # takes 2
    _test_choose( TestHotspot( 5, 5 ), cleft.RIGHT_WAVE) # takes 2

    # worn immediate
    _test_choose( TestHotspot( 4, 1 ), cleft.WORN_POSTURE) # immediate
    _test_choose( TestHotspot( 1, 1 ), cleft.WORN_POSTURE) # again/continue



    print("choose_animation passed")

def _test_choose( hotspot, expect, positive_sense = True ):
    #print()
    rez = cleft.choose_animation( hotspot ) 
    message = "expected {} '{}', saw '{}' history {} was {} -> now {}".format(
        ("" if positive_sense else "!"), expect, rez, cleft.choose_animation_raw_history,
            hotspot.was,
            hotspot.now
        )
    if positive_sense:
        assert rez == expect,message
    else:
        assert rez != expect,message
    print("ok: " + message)

def history_reset( all=cleft.IDLE, *rest):
    cleft.choose_animation_raw_history = Ring( len(cleft.choose_animation_raw_history), all)
    if len(rest) == 0:
        return
    cleft.choose_animation_raw_history.insert(all) # first
    for v in rest:
        cleft.choose_animation_raw_history.insert(v)



original = copy( cleft.choose_animation_raw_history )
test_if_ntimes()
cleft.choose_animation_raw_history = copy( original)
test_choose_animation()

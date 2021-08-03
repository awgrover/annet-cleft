#pragma once /* full sin wave that moves from each tip

*/

#include "every.h"
#include "begin_run.h"
#include "AccelStepperShift.h"

class AnimationWave1  : public BeginRun {
  public:
    AccelStepperNoted** const motors; // [] of AccelStepper
    const float amplitude; // in meters
    const float cycle_fraction; // amount of full cycle that fits in 1/2 motor_ct
    const float frequency; // in cycles/sec

    AnimationWave1(AccelStepperNoted** motors, float amplitude, float cycle_fraction, float frequency)
      : motors(motors),
        amplitude(amplitude),
        cycle_fraction(cycle_fraction),
        frequency(frequency)
    {
    }

    void begin() {
    }

    boolean run() {
      return true; // IFF working
    }
};

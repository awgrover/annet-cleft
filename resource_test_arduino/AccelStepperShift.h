#pragma once
#include <AccelStepper.h> // Mike McCauley <mikem@airspayce.com>

class AccelStepperShift : public AccelStepper {
  public:
    void step(long step_phase) {

      switch (step_phase & 0x3)
      {
        case 0:    // 1010
          setOutputPins(0b0101);
          break;

        case 1:    // 0110
          setOutputPins(0b0110);
          break;

        case 2:    //0101
          setOutputPins(0b1010);
          break;

        case 3:    //1001
          setOutputPins(0b1001);
          break;
      }


    }
};

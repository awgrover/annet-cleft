#pragma once

// NOT FINISHED -- NOT USED YET

#include "ExponentialSmooth.h"

class ArrayAnimation : public BeginRun {
  public:
    using Heights = float; // our positions are in meters
    
    class Positions; // see below

    int position_ct;
    int loop_rate=1;
    ExponentialSmooth<unsigned long> framerate = ExponentialSmooth<unsigned long>(2); // in millis()/loop
    unsigned long last_loop_time = 1000; // start with something reasonable
    boolean first_time = true;
    
    ArrayAnimation(int position_ct)
      : position_ct(position_ct)
    {}

    void begin() {}
    
    boolean run() {
      unsigned long last_time;
      // assume we are running an animation
      // track the loop-rate, which is the millis/step
      // If we hit target position (from last animation target)
      //  the motors do 1 step / loop at most
      //  update actual-loop-rate-average
      //  calculate next target:
      //    for about 100 micros (frames/sec)
      //  set the desired speed

      // a key-frame has a desired speed
      //  a sin has many pieces at varying speeds 
      // a key-frame is a target position
      //  the sin's inflections are 0 speed
      // move to target, maxspeed, at accel
      // but we need a PAUSE frame, with no step #

      // alt sin:
      // moveto w/ accel/decel, 0 at end
      // repeat
      // can't: we work in deltas
      
      // if a loop takes a small amount of time
      // then we could do: 
      //  if frame expired, setup next frame 
      //    but, we'd drift
      // if a loop takes a large time
      // then
      //  we won't get called anywhere near the frame-length
      //  and we'll tend to "fire" every 2 loops
      //  which will be closer to 2 frames
      // Use the last actual frame-length (frame+loop)
      //  as the predictor for the next frame-length
      //  miminum "min-frame-length", i.e. 10 frames/sec
      // Compute movement for 2x frame-length
      //  to allow unpredictable, and longer loops
      // We'll get called before 2x
      //  probably around +- 10%
      
      // 
      if (first_time) {
        last_time = millis() - 1000; // pretend 1sec frame
        first_time = false;
      }
      
      // absolute positions are in the motors
      Heights delta[position_ct];

      return true; // if we have an animation...
    }
    
    // Patterns
    ArrayAnimation &goto_zero_plane() { // WRONG, state chagne probaby
      // default init
      //static Positions a[position_ct];// = new ArrayAnimation();
      return *this;
    }
};

class ArrayAnimation::Positions  {
    // just a list of heights as the target
  public:

    int position_ct;
    Heights* position = NULL;

    Positions(int position_ct)
      : position_ct(position_ct)
    {
      position = new Heights[position_ct];
      for (int i = 0; i < position_ct; i++) position[i] = 0.0f;
    }

    // default shallow copy cons

    // Interface
    void begin( Heights current_position[] ) {
      // we are going to store our total delta from current_position
      // assuming we want target move in 1 frame
      for (int i = 0; i < position_ct; i++) {
        position[i] = position[i] - current_position[i];
      }
    }


    void update( unsigned long frame_duration, Heights* delta) {
      // fill in delta[]'s, for a frame of frame_duration msecs
      // we assume a base rate of 1 frame / sec
      float frame_fraction = frame_duration / 1000.0;
      for (int i = 0; i < position_ct; i++) {
        // will get round off errors accumulating
        delta[i] = position[i] * frame_fraction;
      }
    }


};

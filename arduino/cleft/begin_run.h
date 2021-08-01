#pragma once

class BeginRun {
    // protocol for .begin() and .run() on a system

  public:
    virtual void begin() = 0;
    virtual boolean run() = 0; // should return true if you are "running"

    class Noop; // the noop version, subclass of BeginRun
};

class BeginRun::Noop : public BeginRun {
    // because we can't have a zero-length array
  public:
    virtual void begin() {};
    virtual void loop() {
      delay(20);  // so integrated usb can respond to upload
    };
};

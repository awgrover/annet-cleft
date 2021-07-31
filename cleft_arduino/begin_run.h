#pragma once

class BeginRun {
    // protocol for .begin() and .run() on a system

  public:
    virtual void begin() = 0;
    virtual boolean run() = 0; // should return true if you are "running"
};

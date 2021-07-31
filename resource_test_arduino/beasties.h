#pragma once

class Beastie {
    // setup(), begin(), loop() protocol
    // for testing resource usage
  public:
    virtual void setup() = 0;
    virtual void begin() = 0;
    virtual void loop() = 0;
};

#pragma once

class NoopBeastie : public Beastie {
  // because we can't have a zero-length array
  public:
    // setup(), begin(), loop() protocol
    virtual void setup() {};
    virtual void begin() {};
    virtual void loop() {
      delay(20);  // so integrated usb can respond to upload
    };
};

#pragma once

class Beastie {
  public:
  // setup(), begin(), loop() protocol
  virtual void setup() = 0;
  virtual void begin() = 0;
  virtual void loop() = 0;
};

#pragma once

class NoopBeastie : public Beastie {
  public:
  // setup(), begin(), loop() protocol
  virtual void setup() {};
  virtual void begin() {};
  virtual void loop() { delay(20); }; // so integrate usb can respond to upload
};

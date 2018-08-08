#include "Wire.h"
#include "hardwareserial.h"

class HIH7120{
public:
  void update();
  double humidity;
  double cTemp;
  double fTemp;
private:
  int address = 0x27;
  uint8_t data[4];
  uint8_t status;
  unsigned long readTimeout = 100;
  unsigned long startTime;
};

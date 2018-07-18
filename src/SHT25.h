#include "Wire.h"
#include "hardwareserial.h"

class SHT25{
public:
  void init();
  double readHumidity();
  double readTemp(char unit);
private:
  int address = 0x40;
  double humidity;
  double cTemp;
  double fTemp;
  uint8_t data[2];
  uint8_t status;
  unsigned long readTimeout = 600;
  unsigned long startTime;
};

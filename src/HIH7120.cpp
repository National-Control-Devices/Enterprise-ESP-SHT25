#include "HIH7120.h"

void HIH7120::update(){
  digitalWrite(17, HIGH);
  delay(10);
  Wire.beginTransmission(address);
  Wire.write(0x00);
  status = Wire.endTransmission();
  if(status !=0){
    Serial.println("Sensor coms failed");
    digitalWrite(17, LOW);
    return;
  }
  Wire.requestFrom(address, 4);
  startTime = millis();
  while(Wire.available() < 4 && millis() < startTime + readTimeout);
  if (Wire.available() < 4){
    Serial.println("Sensor did not respond");
    digitalWrite(17, LOW);
    return;
  }
  data[0] = Wire.read();
  data[1] = Wire.read();
  data[2] = Wire.read();
  data[3] = Wire.read();
  digitalWrite(17, LOW);

  int reading_hum = (data[0] << 8) + data[1];
  humidity = reading_hum / 16382.0 * 100.0;

  int reading_temp = (data[2] << 6) + (data[3] >> 2);
  cTemp = reading_temp / 16382.0 * 165.0 - 40;
  fTemp = (cTemp * 1.8) + 32;
  return;
}

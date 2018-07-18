#include "SHT25.h"

void SHT25::init(){
  pinMode(17, OUTPUT);
  Wire.begin();
}

double SHT25::readHumidity(){
  digitalWrite(17, HIGH);
  delay(10);
  Wire.beginTransmission(address);
  Wire.write(0xF5);
  status = Wire.endTransmission();
  if(status !=0){
    Serial.println("Sensor coms failed");
    return 0.00;
  }
  delay(100);
  Wire.requestFrom(address, 2);
  startTime = millis();
  while(Wire.available() < 2 && millis() < startTime + readTimeout);
  if (Wire.available() < 2){
    Serial.println("Sensor did not respond");
    return 0.00;
  }
  data[0] = Wire.read();
  data[1] = Wire.read();

  humidity = ((((data[0] * 256.0) + data[1]) * 125.0) / 65536.0) - 6;
  digitalWrite(17, LOW);
  // Serial.printf("Humidity: %f \n", humidity);
  return humidity;
}

double SHT25::readTemp(char unit){
  digitalWrite(17, HIGH);
  delay(10);
  Wire.beginTransmission(address);
  Wire.write(0xF3);
  status = Wire.endTransmission();
  if(status !=0){
    Serial.println("Sensor coms failed");
    digitalWrite(17, LOW);
    return 0.00;
  }
  delay(100);
  Wire.requestFrom(address, 2);
  startTime = millis();
  while(Wire.available() < 2 && millis() < startTime + readTimeout);
  if (Wire.available() < 2){
    Serial.println("Sensor did not respond");
    digitalWrite(17, LOW);
    return 0.00;
  }
  data[0] = Wire.read();
  data[1] = Wire.read();

  cTemp = ((((data[0] * 256.0) + data[1]) * 175.72) / 65536.0) - 46.85;
  fTemp = (cTemp * 1.8) + 32;
  if(unit == 'f'){
    digitalWrite(17, LOW);
    return fTemp;
  }else{
    digitalWrite(17, LOW);
    return cTemp;
  }
}

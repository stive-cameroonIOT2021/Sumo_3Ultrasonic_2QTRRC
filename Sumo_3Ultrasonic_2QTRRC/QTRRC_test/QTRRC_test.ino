#include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 2;
uint16_t sensorValues[SensorCount];

void setup() {
  // put your setup code here, to run once:
   Serial.begin(9600);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){7, 8}, SensorCount);//Left, Right
  qtr.setEmitterPin(2);
}

void loop() {
  // put your main code here, to run repeatedly:
  qtr.read(sensorValues);
for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  
Serial.println();
}

#include <Wire.h>

void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
}

void loop()
{
  Wire.requestFrom(8, 2);    // request 6 bytes from slave device #8
  Serial.println(getHeading());
  delay(500);
}

float getHeading(){
  Wire.requestFrom(8, 2);    // request 6 bytes from slave device #8
  byte MSB = Wire.read();
  byte LSB = Wire.read();
  return (MSB<<8)|LSB;
}

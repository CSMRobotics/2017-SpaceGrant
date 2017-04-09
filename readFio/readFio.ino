#include <Wire.h>

void setup() {
  // put your setup code here, to run once:
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(57600);  // start serial for output
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.requestFrom(8, 2);    // request 2 bytes from slave device #8

  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    Serial.print(c);         // print the character
  }

  delay(500);
}

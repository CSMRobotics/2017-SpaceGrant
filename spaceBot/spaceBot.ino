// Ann Gustafson, Colby Moxham, Nhan Tran, John Wiens - Spring 2017
// Motor control on Arduino
// Currently moves and has ultrasonics.
// Also has IMU.
#include <NewPing.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
// The Sparkfun Library can be found here:
// https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library

// The New Ping Library Can be found here:
//https://bitbucket.org/teckel12/arduino-new-ping/downloads/

//Ultrasonic pins and max distance they can read
#define SONAR_NUM 3 //Number of ultrasonics
#define PING_INTERVAL 33 //33 ms per sensor
#define TRIGGER_PIN_LEFT 7
#define ECHO_PIN_LEFT 6
#define TRIGGER_PIN_RIGHT 5
#define ECHO_PIN_RIGHT 4
#define TRIGGER_PIN_FRONT 3
#define ECHO_PIN_FRONT 2
#define MAX_DISTANCE 400

//Magnometer Declination
#define DECLINATION -8.58
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

//Distance we want to alert robot
#define HITTING_DISTANCE 40

unsigned long pingTimer[SONAR_NUM]; //When each pings
unsigned int cm[SONAR_NUM]; //Store ping distances
uint8_t currentSensor = 0; //Which sensor is currently going

//The left and right ultrasonics
//NewPing sonarLeft(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE);
//NewPing sonarRight(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE);
//NewPing sonarFront(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);

//Array of sensors
NewPing sonar[SONAR_NUM] = {
    NewPing(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE),
    NewPing(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE),
    NewPing(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE)
  };

//Motor direction and PWM pins
int motorLeftDir = 8;
int motorRightDir = 9;
int motorLeftPwm = 10;
int motorRightPwm = 11;

LSM9DS1 imu;

void setup()
{
  Serial.begin(115200);
  
  pinMode(motorLeftDir, OUTPUT);
  pinMode(motorRightDir, OUTPUT);
  pinMode(motorLeftPwm, OUTPUT);
  pinMode(motorRightPwm, OUTPUT);

  pingTimer[0] = millis() + 75; // First ping start in ms.
  for (uint8_t i = 1; i < SONAR_NUM; i++)
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
}

void loop()
{
  delay(500);  //Not sure if we'll actually want the delay here
  //The distances from the left and right sensors
 /* unsigned int distanceL = sonarLeft.ping_cm();
  unsigned int distanceR = sonarRight.ping_cm();
  
  //This will check how close we are to the alotted distance from the object and
  //change course accordingly
  if (distanceL <= HITTING_DISTANCE || distanceR <= HITTING_DISTANCE)
  {
      brake();
      checkLeftUltra(distanceL);
      distanceR = sonarRight.ping_cm();
      checkRightUltra(distanceR);
      distanceL = sonarLeft.ping_cm();
  }*/
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if (millis() >= pingTimer[i]) {
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;
      if (i == 0 && currentSensor == SONAR_NUM - 1)
        oneSensorCycle(); // Do something with results.
      sonar[currentSensor].timer_stop();
      currentSensor = i;
      cm[currentSensor] = sonar[currentSensor].ping_cm();
      sonar[currentSensor].ping_timer(echoCheck);
    }
  }
//  Serial.print("Left");Serial.println(sonar[0].ping_cm());
//  Serial.print("Right");Serial.println(sonar[1].ping_cm());
//  Serial.print("Front");Serial.println(sonar[2].ping_cm());
  forward();
}

//MOVEMENT FUNCTIONS
void forward()
{
  digitalWrite(motorLeftDir, HIGH);
  digitalWrite(motorRightDir, HIGH);
  analogWrite(motorLeftPwm, 255);
  analogWrite(motorRightPwm, 255);
}

void backward()
{
  digitalWrite(motorLeftDir, LOW);
  digitalWrite(motorRightDir, LOW);
  analogWrite(motorLeftPwm, 255);
  analogWrite(motorRightPwm, 255);
}

void rightward()
{
  digitalWrite(motorLeftDir, HIGH);
  digitalWrite(motorRightDir, LOW);
  analogWrite(motorLeftPwm, 255);
  analogWrite(motorRightPwm, 255);
}

void leftward()
{
  digitalWrite(motorLeftDir, LOW);
  digitalWrite(motorRightDir, HIGH);
  analogWrite(motorLeftPwm, 255);
  analogWrite(motorRightPwm, 255);
}

void brake()
{
  digitalWrite(motorLeftDir, HIGH);
  digitalWrite(motorRightDir, HIGH);
  analogWrite(motorLeftPwm, 0);
  analogWrite(motorRightPwm, 0);
}

void setMotor(int motorPin,int dirPin, double power){
  if (power > 1){
    power = 1;
  }
  else if (power < -1){
    power = -1;
  }
  
  if (power > 0){
    digitalWrite(dirPin,HIGH);
  }
  else{
    digitalWrite(dirPin,LOW);
  }
  
  analogWrite(motorPin, abs(power)*255);
}


//ULTRASONIC FUNCTIONS
void echoCheck() { // If ping echo, set distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}
 
void oneSensorCycle() { // Do something with the results.
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    Serial.print(i);
    Serial.print("=");
    Serial.print(cm[i]);
    Serial.print("cm \n");
  }
//  Serial.println(cm[0]);
}

/*void checkLeftUltra(unsigned int dist)
{
  while (dist <= HITTING_DISTANCE)
  {
    rightward();
    dist = sonarLeft.ping_cm();
  }
}

void checkRightUltra(unsigned int dist)
{
  while (dist <= HITTING_DISTANCE)
  {
    leftward();
    dist = sonarRight.ping_cm();
  }
}*/

//IMU Functions
void updateIMU(){
  if (imu.gyroAvailable()){
    imu.readGyro();  
  }
  if ( imu.accelAvailable()){
    imu.readAccel();
  }
  if ( imu.magAvailable()){
    imu.readMag();
  }
}

float getIMUHeading(){
  updateIMU();
  float heading;
  if (imu.my == 0)
    heading = (imu.mx < 0) ? PI : 0;
  else
    heading = atan2(imu.mx, imu.my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  return heading;
}



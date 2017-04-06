// Ann Gustafson - Spring 2017
// Motor control on Arduino
// Currently moves and has ultrasonics.
#include <NewPing.h>

//Ultrasonic pins and max distance they can read
#define TRIGGER_PIN_LEFT 7
#define ECHO_PIN_LEFT 6
#define TRIGGER_PIN_RIGHT 5
#define ECHO_PIN_RIGHT 4
#define MAX_DISTANCE 400

//Distance we want to alert robot
#define HITTING_DISTANCE 15

//The left and right ultrasonics
NewPing sonarLeft(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE);
NewPing sonarRight(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE);

//Motor direction and PWM pins
int motorLeftDir = 8;
int motorRightDir = 9;
int motorLeftPwm = 10;
int motorRightPwm = 11;

void setup()
{
  Serial.begin(115200);
  
  pinMode(motorLeftDir, OUTPUT);
  pinMode(motorRightDir, OUTPUT);
  pinMode(motorLeftPwm, OUTPUT);
  pinMode(motorRightPwm, OUTPUT);
}

void loop()
{
  delay(50);  //Not sure if we'll actually want the delay here
  //The distances from the left and right sensors
  unsigned int distanceL = sonarLeft.ping_cm();
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
  }
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

//ULTRASONIC FUNCTIONS
void checkLeftUltra(unsigned int dist)
{
  while (dist <= HITTING_DISTANCE)
  {
    rightward();
    dist = sonarLeft.ping_cm();
  }
}

void checkRightUltra(unsigned int dist)
{
  while (distanceR <= HITTING_DISTANCE)
  {
    leftward();
    dist = sonarRight.ping_cm();
  }
}



#include "Arduino.h"
#include "PID.h"

PID::PID(double kp, double ki, double kd){
  PValue = kp;
  IValue = ki;
  DValue = kd;
  maxOutput = 1;
  minOutput = 1;
}

double PID::update(double value){
  error = setPoint - value;
  if ((error * PValue < maxOutput) && (error * PValue > minOutput)) {
    totalError += error;
  } 
  else {
    totalError = 0;
  }
    result = (PValue * error + IValue * totalError + DValue * (error - prevError));
    prevError = error;
    return result;
}
double PID::getResult(){
return result;
}

void PID::setTarget(double target){
  setPoint = target;
}

void PID::setPID(double kp, double ki, double kd){
  PValue = kp;
  IValue = ki;	
  DValue = kd;
}


  



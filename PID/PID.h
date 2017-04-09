
#ifndef PID_h
#define PID_h

#include "Arduino.h"

class PID
{
  public:
    PID(double kP, double kI, double kD);
    void setPID(double kP, double kI, double kD);
    double getResult();
    double update(double value);
    void setTarget(double target);
   private:
     double error;
     double prevError;
     double totalError;
     double PValue;
     double IValue;
     double DValue;
     double setPoint;
     double output;
     double result;
     double maxOutput;
     double minOutput;
};

#endif

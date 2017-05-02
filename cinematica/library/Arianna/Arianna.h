#ifndef Arianna_h
#define Arianna_h

#include "Arduino.h"
#include <AccelStepper.h>
#include <MultiStepper.h>

#define RAD2DEG(_val) _val*180/PI
enum Verso{ CK, ACK};
//http://www.nongnu.org/avr-libc/user-manual/group__avr__math.html
class Arianna
{
  public:
    Arianna(int STEPSX,int DIRSX,int STEPDX,int DIRDX,int ENABLE);
    void setGeometry(float resolutionOfOneStepInDeg, uint16_t diameterOfWheel, uint16_t  distanceBetweenWheel);
    void circle(float alpha,long r0, Verso v);
    void line(long _pos);
    void move(long x,long y);
    void runToDistance();
    long* getCurrentPosition();
    float getCurrentAngle();
    //void runSingleStep();
    //long stepBeforeTarget();
    int DELTAWHEEL;
    float STEPONMM;
  private:
    AccelStepper _SxMotor;
    AccelStepper _DxMotor;
    MultiStepper _steppers;
    long _currentXposition;
    long _currentYposition;
    long _positions[2];
    float _currentAngle;
    void _circle(float alpha,long r0, Verso v);
    void _line(long _pos);
    void _setDistance();
    long _TOSTEP(long t);
    //long _TOSTEP(float t);
    //int _STEPSX,_DIRSX,_STEPDX,_DIRDX,_ENABLE;
    float angleBetween();
};

#endif

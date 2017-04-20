#include <AccelStepper.h>
#include <MultiStepper.h> //http://www.airspayce.com/mikem/arduino/AccelStepper/
#define SXDIR 6
#define SXSTEP 7
#define DXDIR 4
#define DXSTEP 5
#define ENABLE 3

#define AVANTI true
#define INDIETRO ( !AVANTI )

#define evalDir(state,stepper) ((stepper==SXDIR)? state : !state)
#define DEG 1.8
#define NUMSTEP 360/DEG
#define CIRCONFERENCE 2*44.8*3.1418/1000 //deve essere in metri
#define STEPONMM NUMSTEP/((double)CIRCONFERENCE)


AccelStepper SxMotor(AccelStepper::DRIVER,SXSTEP,SXDIR);//low to high transition make step
AccelStepper DxMotor(AccelStepper::DRIVER,DXSTEP,DXDIR);

MultiStepper steppers;

void setupMotori(){
  //setup the single proprierty of stepper
  SxMotor.setMaxSpeed(800);
  DxMotor.setMaxSpeed(800);
  SxMotor.setAcceleration(200);
  DxMotor.setAcceleration(200);
  
  //build the array (up to 10 stepper) that work toghter
  steppers.addStepper(SxMotor);
  steppers.addStepper(DxMotor);
  
  pinMode(ENABLE,OUTPUT);
  digitalWrite(ENABLE,HIGH);
}

void setup() {
 setupMotori();
 Serial.begin(9600);
}

void loop() {
   int s;
  int d;
  byte c;
  while (Serial.available() > 0) {
    c = Serial.read();
    switch(c){
      case 'r': long positions[2];
                positions[0]=Serial.parseInt();
                positions[1]=Serial.parseInt();
                steppers.moveTo(positions);
                steppers.runSpeedToPosition();//opppure run per il single step
                break;
     default: Serial.print("NON compriendo");
    }

  }
}


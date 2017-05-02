#include <AccelStepper.h>

#define SXDIR 6
#define SXSTEP 7
#define DXDIR 4
#define DXSTEP 5
#define ENABLE 3

//il primo bit è spiegato qui cinematica/dependencies/AccelStepper/doc/classAccelStepper.html
//definisce la tipologia di stepper (driver tipo A4988 step/dir oppure 2 fili / 4(bipolari))

AccelStepper SxMotor = AccelStepper(1,SXSTEP,SXDIR);//low to high transition make step
AccelStepper DxMotor = AccelStepper(1,DXSTEP,DXDIR);

void setup() {
  Serial.begin(115200);
  SxMotor.setMaxSpeed(400000);
  DxMotor.setMaxSpeed(400000);
  SxMotor.setAcceleration(3000);
  DxMotor.setAcceleration(3000);
  
}

void loop() {
  int s;
  int d;
  byte c;
  long tempo;
  while (Serial.available() > 0) {
    c = Serial.read();
    switch(c){
      case 's': SxMotor.run(); break;
      case 'd': SxMotor.run(); break;
      case 't': SxMotor.run(); DxMotor.run(); break;
      case 'n': 
                s=Serial.parseInt();
                d=Serial.parseInt();
                SxMotor.move(s);
                DxMotor.move(d);
                break;
      case 'a': 
                while(DxMotor.distanceToGo()!= 0 || SxMotor.distanceToGo() != 0){
                  SxMotor.run();
                  DxMotor.run();
                  Serial.println( millis()- tempo);
                  tempo=millis();
                }
                break;
//      default: Serial.print("NON compriendo");
    }

  }

}

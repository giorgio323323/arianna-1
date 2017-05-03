#include <Arianna.h>

int SXDIR = 6, SXSTEP = 7, DXDIR = 4, DXSTEP = 5, ENABLE = 3;

const float DEG = 0.1125;
const unsigned int DIAMETER = 72;
const unsigned int DELTAWHEEL = 172; //mm


Arianna arianna(SXSTEP, SXDIR, DXSTEP, DXDIR, ENABLE);
//Arianna arianna = new Arianna(SXSTEP, SXDIR, DXSTEP, DXDIR, ENABLE);
#define DEBUG //for debug purprose

void setup() {
  arianna.setGeometry(DEG, DIAMETER, DELTAWHEEL);
  Serial.begin(9600);
}

void loop() {
  int s;
  int d;
  float f;
  int8_t c;
  while (Serial.available() >= 2) {//ATTENZIONE la seriale e il codice viaggiano a velocità differenti, da ora in poi ogni riga dovrà avere un minimo di 2 byte
    c = Serial.read();
    switch (c) {
      case 'm':
        s = Serial.parseInt(); //delta x
        d = Serial.parseInt(); //delta y
        arianna.move(s, d);
        break;
      case 'M':
        switch (Serial.parseInt()) {
          case 114:
            Serial.print("Current position: "); Serial.print((long)(arianna.getCurrentPosition())[0]); Serial.print(" "); Serial.println((long)(arianna.getCurrentPosition())[1]);
            break;
          case 115:
            Serial.print("Current angle: "); Serial.println(arianna.getCurrentAngle());
        } 
        break;
      case 'G':
        switch (Serial.parseInt()) {
          case 0:
          case 1:
            s = Serial.parseInt(); //delta x
            d = Serial.parseInt(); //delta y
            arianna.move(s, d);
            break;
        }

        break;
      case 'l': s=Serial.parseInt(); arianna.line(s);//feed mm
        break;
      case 'c': c = Serial.read();
#ifdef DEBUG
        Serial.println(c);
#endif
        f = Serial.parseFloat();
        s = Serial.parseInt();
        if (c == 99) { //ca 90 30 per eseguire una rotazione a sinistra (anticlockwise) di 90 gradi con la ruota sinistra che esegue una circonferenza con centro distante 30 mm
          //TODO BUG there is no assumption with the order of the parseInt and parseFloat, please notify TODO TODO in this function circle(Serial.parseFloat(),Serial.parseInt(),174,positions,Verso::ACK);
          arianna.circle(f, s, CK); //feed mm,verso orario //TODO cambiare con const
        } else if (c == 'a') {
          arianna.circle(f, s, ACK); //verso antiorario
        }
        break;
      default: Serial.print("NON compriendo");// Serial.flush();
    }

  }
}


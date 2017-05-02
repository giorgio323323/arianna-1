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
#define DEG 0.1125
#define NUMSTEP 360/DEG
#define CIRCONFERENCE 72*PI //mm
#define STEPONMM NUMSTEP/((double)CIRCONFERENCE)
#define TOSTEP(val) (STEPONMM * val)
const unsigned int DELTAWHEEL= 172; //mm

//orario anti orario, viene tradotto in "un bit"
enum Verso{ CK, ACK};

AccelStepper SxMotor(AccelStepper::DRIVER,SXSTEP,SXDIR);//low to high transition make step
AccelStepper DxMotor(AccelStepper::DRIVER,DXSTEP,DXDIR);

long positions[2];
MultiStepper steppers;


#define DEBUG //for debug purprose

void setupMotori(){
  //setup the single proprierty of stepper
  SxMotor.setMaxSpeed(800);
  DxMotor.setMaxSpeed(800);
  SxMotor.setAcceleration(200);
  DxMotor.setAcceleration(200);
  
  //build the array (up to 10 stepper) that work toghter
  steppers.addStepper(SxMotor);
  steppers.addStepper(DxMotor);
  positions[0] = 0;// la libreria usa cordinate assolute, in futuro si sistema
  positions[1] = 0;
  pinMode(ENABLE,OUTPUT);
  digitalWrite(ENABLE,HIGH);
}

void setup() {
 setupMotori();
 Serial.begin(9600);
Serial.print("Step on mm: ");
Serial.println(STEPONMM);
Serial.print("COnverted : ");
Serial.println(TOSTEP(DELTAWHEEL));
Serial.print("CIRCONFERENCE : ");
Serial.println(TOSTEP(CIRCONFERENCE));
}

void line(long _pos,long* pos){
  pos[0]+=TOSTEP(_pos);
  pos[1]+=TOSTEP(_pos);
}
//s1 è lo spazioe seguito dalla ruota sinistra :
//nel caso di verso antiorario positivo il centro della circonferenza è a sinistra della ruota sinistra (il roboto gira verso sinistra)
//nel caso negativo gira a destra (il centro della circonferenza seguita dalla ruota è verso il centro del robot, a destra della ruota sinistra)

//a seconda che l'angolo sia maggiore o minore di zero potrò eseguire movimenti in avanti o indietro sulla circonferenza
void circle(float alpha,long r0,float delta,long* pos, Verso v){ 
 #ifdef DEBUG
Serial.print("Circle!! With: alpha: ");
Serial.println(alpha);
Serial.print("r0 : ");
Serial.println(r0);
Serial.print("delta : ");
Serial.println(delta);
Serial.print("Verso : ");
Serial.println(v);
#endif  
  long s1,s2;
  int dir = (alpha >=0)?1:-1;
  /*switch(v){
    
  }*/
  if(v==ACK){
      s2=TOSTEP(dir*alpha*2*3.14*(r0 + delta)/360);//TODO UTILIZZARE DEFINE
      s1=TOSTEP(dir*alpha*2*3.14*r0/360);
  }
  else{
      s2=TOSTEP(dir*alpha*2*3.14*(r0)/360);//TODO UTILIZZARE DEFINE
      s1=TOSTEP(dir*alpha*2*3.14*(r0 + delta)/360);
  }
#ifdef DEBUG
Serial.print("s1 : ");
Serial.println(s1);
Serial.print("s2 : ");
Serial.println(s2);
#endif
pos[0]+=s1;
    pos[1]+=s2;
}
void loop() {
   int s;
  int d;
  float f;
  int8_t c;
  while (Serial.available() >= 2) {//ATTENZIONE la seriale e il codice viaggiano a velocità differenti, da ora in poi ogni riga dovrà avere un minimo di 2 byte
    c = Serial.read();
    switch(c){
      case 'l': line(Serial.parseInt(),positions);//feed mm
                steppers.moveTo(positions);
                steppers.runSpeedToPosition();//opppure run per il single step
                break;
      case 'c': c=Serial.read();
                Serial.println(c);
                f=Serial.parseFloat();
                s=Serial.parseInt();
                if(c==99){//ca 90 30 per eseguire una rotazione a sinistra (anticlockwise) di 90 gradi con la ruota sinistra che esegue una circonferenza con centro distante 30 mm
                 //TODO BUG there is no assumption with the order of the parseInt and parseFloat, please notify TODO TODO in this function circle(Serial.parseFloat(),Serial.parseInt(),174,positions,Verso::ACK); 
                 circle(f,s,DELTAWHEEL,positions,CK); //feed mm,verso orario //TODO cambiare con const
                 #ifdef DEBUG 
                 Serial.println("CK nel loop");
                 #endif
                }else if(c=='a'){
                 circle(f,s,DELTAWHEEL,positions,ACK); //verso antiorario
                 #ifdef DEBUG 
                 Serial.println("ACK nel loop"); 
                 #endif
                }
                steppers.moveTo(positions);
                steppers.runSpeedToPosition();//opppure run per il single step
                break;
     default: Serial.print("NON compriendo");
    }

  }
}


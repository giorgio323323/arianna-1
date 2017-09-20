
#include <AccelStepper.h>
#include <MultiStepper.h>

#define RAD2DEG(_val) _val*180/PI
enum Verso{ CK, ACK};
enum Mode{ABS,REL};//modalità G0-1

int SXDIR = 6, SXSTEP = 7, DXDIR = 4, DXSTEP = 5, ENABLE = 3;

const float DEG = 0.1125;
const unsigned int DIAMETER = 72;
const unsigned int DELTAWHEEL = 172; //mm


//Arianna arianna(SXSTEP, SXDIR, DXSTEP, DXDIR, ENABLE);

#define DEBUG //for debug purprose
//PUBLIC
    int _DELTAWHEEL;
    float _STEPONMM;
//PRIVATE 
AccelStepper _SxMotor(AccelStepper::DRIVER,SXSTEP,SXDIR);//low to high transition make step
AccelStepper _DxMotor(AccelStepper::DRIVER,DXSTEP,DXDIR);
 MultiStepper _steppers;
    long _currentXposition;
    long _currentYposition;
    long _positions[2];
    float _currentAngle;
    //long _TOSTEP(float t);
    //int _STEPSX,_DIRSX,_STEPDX,_DIRDX,_ENABLE;
    float angleBetween();
    Mode mode;

void setupMotor(){
  _SxMotor.setMaxSpeed(500);//TODO manca funzione che lo fa
  _DxMotor.setMaxSpeed(500);
  _SxMotor.setAcceleration(200);
  _DxMotor.setAcceleration(200);
  
  //build the array (up to 10 stepper) that work toghter
  _steppers.addStepper(_SxMotor);
  _steppers.addStepper(_DxMotor);
  _positions[0] = 0;// la libreria usa cordinate assolute, in futuro si sistema
  _positions[1] = 0;
  pinMode(ENABLE,OUTPUT);
  digitalWrite(ENABLE,HIGH);

  
  _currentXposition=0;
  _currentYposition=0;
  _currentAngle=0;

  mode=ABS;
  }
void setup() {
  setupMotor();
  //da fare in una funz separata
  setGeometry(DEG, DIAMETER, DELTAWHEEL);
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
      case 'M':
        switch (Serial.parseInt()) {
          case 114:
            Serial.print(F("Current position: ")); Serial.print((long)(getCurrentPosition())[0]); Serial.print(F(" ")); Serial.println((long)(getCurrentPosition())[1]);
            break;
          case 115:
            Serial.print(F("Current angle: ")); Serial.println(getCurrentAngle());
        } 
        break;
      case 'G':
        switch (Serial.parseInt()) {
          case 0:
          case 1:
            s = Serial.parseInt(); //delta x
            d = Serial.parseInt(); //delta y
            if(mode==ABS)//di default è assoluto, guardare setup 
              move(s, d,_currentAngle,&_currentXposition,&_currentYposition);//chiamata all'assoluto
            else if(mode==REL)
              move(s,d,0);//chiama al relativo
            break;
           case 90: Serial.println(F("Setted to absolute position"));
            mode=ABS;
            break;
           case 91: Serial.println(F("Setted to relative position"));
            mode=REL;
            break;
        }

        break;
      case 'l': s=Serial.parseInt(); line(s);//feed mm
        break;
      case 'c': c = Serial.read();
#ifdef DEBUG
        Serial.println(c);
#endif
        f = Serial.parseFloat();
        s = Serial.parseInt();
        if (c == 99) { //ca 90 30 per eseguire una rotazione a sinistra (anticlockwise) di 90 gradi con la ruota sinistra che esegue una circonferenza con centro distante 30 mm
          //TODO BUG there is no assumption with the order of the parseInt and parseFloat, please notify TODO TODO in this function circle(Serial.parseFloat(),Serial.parseInt(),174,positions,Verso::ACK);
          circle(f, s, CK); //feed mm,verso orario //TODO cambiare con const
        } else if (c == 'a') {
          circle(f, s, ACK); //verso antiorario
        }
        break;
      default: Serial.print(F("NON compriendo"));// Serial.flush();
    }

  }
}

void setGeometry(float resolutionOfOneStepInDeg, uint16_t diameterOfWheel, uint16_t  distanceBetweenWheel){
  _STEPONMM=(360/resolutionOfOneStepInDeg)/((float)diameterOfWheel*PI);
  _DELTAWHEEL=distanceBetweenWheel;
}

void circle(float alpha,long r0, Verso v){
  _circle(alpha,r0,v);
  _setDistance();
  runToDistance();
}
void line(long p){
  _line(p);//TODO inserire linee di debug
  _setDistance();
  runToDistance();
}
void move(long x, long y,float angolo,long * currX, long * currY){
  //_positions[0]+=_TOSTEP(x);
  //_positions[1]+=_TOSTEP(y);
  long dX,dY;
  dX=x-*currX;
  dY=y-*currY;
  //TODO REFACTORING CODICE COMUNE
  //http://www.nongnu.org/avr-libc/user-manual/group__avr__math.html
  /*

double atan2  ( double  __y,
double  __x 
)   
The atan2() function computes the principal value of the arc tangent of __y / __x
, using the signs of both arguments to determine the quadrant of the return value. 
The returned value is in the range [-pi, +pi] radians.
  */
  float toAngle = RAD2DEG(atan2(dY,dX));
  float eps = toAngle - angolo;
  _currentAngle=toAngle;
  *currX=x;
  *currY=y;
  
  circle(eps,0,ACK);
  /*if(eps>0)
    cirlce(fabs(eps),0,ACK); //TODO a sinistra andrà bene
  else
    circle(fabs(eps),0,CK); //TODO ERRORE a destra giro intorno alla ruota destra, ERRORE
  */
  line(sqrt(dX*dX + dY*dY));
}

//movimento relativo, non ho bisogno della posizione attuale
void move(long x, long y,float angolo){
  //_positions[0]+=_TOSTEP(x);
  //_positions[1]+=_TOSTEP(y);
  long dX,dY;
  dX=x;
  dY=y;
  
  //http://www.nongnu.org/avr-libc/user-manual/group__avr__math.html
  /*

double atan2  ( double  __y,
double  __x 
)   
The atan2() function computes the principal value of the arc tangent of __y / __x
, using the signs of both arguments to determine the quadrant of the return value. 
The returned value is in the range [-pi, +pi] radians.
  */
  float toAngle = RAD2DEG(atan2(dY,dX));
  circle(angolo,0,ACK);
  /*if(eps>0)
    cirlce(fabs(eps),0,ACK); //TODO a sinistra andrà bene
  else
    circle(fabs(eps),0,CK); //TODO ERRORE a destra giro intorno alla ruota destra, ERRORE
  */
  line(sqrt(dX*dX + dY*dY));
}
void runToDistance(){
  _steppers.runSpeedToPosition();
}

long* getCurrentPosition(){//TODO da sistemare in mm assoluti
  /*long ar[2];
  ar[0]=_currentXposition;
  ar[1]=_currentYposition;
  return ar;*/
  return _positions;
}
float getCurrentAngle(){
  return _currentAngle;
}
 /*
}
4rianna::runSingleStep(){
  _steppers.runSpeedToPosition();
}
*/

 /*
4rianna::stepBeforeTarget(){
  
}
*/
long _TOSTEP(long t){
  return t*_STEPONMM;
}


void _setDistance(){
  _steppers.moveTo(_positions);
}

void _line(long _pos){
  #ifdef DEBUG
  Serial.print("Pianifico linea di:");
  Serial.print(_STEPONMM*_pos);
  Serial.println(" step.");
  #endif

     _positions[0]+=_STEPONMM*_pos;
    _positions[1]+=_STEPONMM*_pos;
                
}
//s1 è lo spazioe seguito dalla ruota sinistra :
//nel caso di verso antiorario positivo il centro della circonferenza è a sinistra della ruota sinistra (il roboto gira verso sinistra)
//nel caso negativo gira a destra (il centro della circonferenza seguita dalla ruota è verso il centro del robot, a destra della ruota sinistra)

//a seconda che l'angolo sia maggiore o minore di zero potrò eseguire movimenti in avanti o indietro sulla circonferenza
void _circle(float alpha,long r0, Verso v){ 
  #ifdef DEBUG
    Serial.print("Circle!! With: alpha: ");
    Serial.println(alpha);
    Serial.print("r0 : ");
    Serial.println(r0);
    Serial.print("delta : ");
    Serial.println(_DELTAWHEEL);
    Serial.print("Verso : ");
    Serial.println(v);
  #endif  
  long s1,s2;
  int dir = (alpha >=0)?1:-1;

  s2=_TOSTEP(dir*alpha*2*3.14*(r0 + _DELTAWHEEL)/360);//TODO UTILIZZARE DEFINE
  s1=_TOSTEP(dir*alpha*2*3.14*r0/360);
  if(v==ACK){
    if(dir>0){
    _positions[0]+=s1;
    _positions[1]+=s2;
    }else{
    _positions[0]-=s1;
    _positions[1]-=s2;
    }
  }
  else{
    if(dir>0){
    _positions[0]+=s2;
    _positions[1]+=s1;
    }else{
    _positions[0]-=s2;
    _positions[1]-=s1;
    }
  }

  #ifdef DEBUG
    Serial.print("s1 : ");
    Serial.println(s1);
    Serial.print("s2 : ");
    Serial.println(s2);
  #endif

}




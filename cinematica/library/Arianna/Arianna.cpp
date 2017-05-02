#include "Arduino.h"
#include "Arianna.h"
//#define DEBUG

Arianna::Arianna(const int STEPSX,const int DIRSX,const int STEPDX,const int DIRDX,const int ENABLE){
  //setup the single proprierty of stepper
 AccelStepper _SxMotor(AccelStepper::DRIVER,STEPSX,DIRSX);//low to high transition make step
AccelStepper _DxMotor(AccelStepper::DRIVER,STEPDX,DIRDX);
  _SxMotor.setMaxSpeed(800);//TODO manca funzione che lo fa
  _DxMotor.setMaxSpeed(800);
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
  _currentAngle=90;
}

void Arianna::setGeometry(float resolutionOfOneStepInDeg, uint16_t diameterOfWheel, uint16_t  distanceBetweenWheel){
	STEPONMM=(360/resolutionOfOneStepInDeg)/((float)diameterOfWheel*PI);
	DELTAWHEEL=distanceBetweenWheel;
}

void Arianna::circle(float alpha,long r0, Verso v){
	_circle(alpha,r0,v);
	_setDistance();
	runToDistance();
}
void Arianna::line(long p){
	_line(p);//TODO inserire linee di debug
	_setDistance();
	runToDistance();
}
void Arianna::move(long x, long y){
  //_positions[0]+=_TOSTEP(x);
  //_positions[1]+=_TOSTEP(y);
	long dX,dY;
	dX=x-_currentXposition;
	dY=y-_currentYposition;
	//http://www.nongnu.org/avr-libc/user-manual/group__avr__math.html
	/*

double atan2	(	double 	__y,
double 	__x 
)		
The atan2() function computes the principal value of the arc tangent of __y / __x
, using the signs of both arguments to determine the quadrant of the return value. 
The returned value is in the range [-pi, +pi] radians.
	*/
	float toAngle = RAD2DEG(atan2(dY,dX));
	float eps = toAngle - _currentAngle;
	circle(eps,0,ACK);
	/*if(eps>0)
		cirlce(fabs(eps),0,ACK); //TODO a sinistra andrà bene
	else
		circle(fabs(eps),0,CK);	//TODO ERRORE a destra giro intorno alla ruota destra, ERRORE
	*/
	line(sqrt(dX*dX + dY*dY));
}

void Arianna::runToDistance(){
	_steppers.runSpeedToPosition();
}

long* Arianna::getCurrentPosition(){//TODO da sistemare in mm assoluti
	/*long ar[2];
	ar[0]=_currentXposition;
	ar[1]=_currentYposition;
	return ar;*/
	return _positions;
}
float Arianna::getCurrentAngle(){
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
long Arianna::_TOSTEP(long t){
	return t*STEPONMM;
}


void Arianna::_setDistance(){
	_steppers.moveTo(_positions);
}

void Arianna::_line(long _pos){
	#ifdef DEBUG
	Serial.print("Pianifico linea di:");
	Serial.print(STEPONMM*_pos);
	Serial.println(" step.");
	#endif
  _positions[0]+=STEPONMM*_pos;	
  _positions[1]+=STEPONMM*_pos;
}
//s1 è lo spazioe seguito dalla ruota sinistra :
//nel caso di verso antiorario positivo il centro della circonferenza è a sinistra della ruota sinistra (il roboto gira verso sinistra)
//nel caso negativo gira a destra (il centro della circonferenza seguita dalla ruota è verso il centro del robot, a destra della ruota sinistra)

//a seconda che l'angolo sia maggiore o minore di zero potrò eseguire movimenti in avanti o indietro sulla circonferenza
void Arianna::_circle(float alpha,long r0, Verso v){ 
	#ifdef DEBUG
		Serial.print("Circle!! With: alpha: ");
		Serial.println(alpha);
		Serial.print("r0 : ");
		Serial.println(r0);
		Serial.print("delta : ");
		Serial.println(DELTAWHEEL);
		Serial.print("Verso : ");
		Serial.println(v);
	#endif  
	long s1,s2;
	int dir = (alpha >=0)?1:-1;

	s2=_TOSTEP(dir*alpha*2*3.14*(r0 + DELTAWHEEL)/360);//TODO UTILIZZARE DEFINE
	s1=_TOSTEP(dir*alpha*2*3.14*r0/360);
	if(v==ACK){
		_positions[0]+=s1;
		_positions[1]+=s2;
	}
	else{
		_positions[0]+=s2;
		_positions[1]+=s1;
	}

	#ifdef DEBUG
		Serial.print("s1 : ");
		Serial.println(s1);
		Serial.print("s2 : ");
		Serial.println(s2);
	#endif
	_positions[0]+=s1;
	_positions[1]+=s2;
}


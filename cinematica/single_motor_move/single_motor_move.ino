// required !!!!!!!!! https://github.com/adafruit/Adafruit-Motor-Shield-library
// required !!!!!!!!! https://github.com/adafruit/Adafruit-Motor-Shield-library

//Positive positions are clockwise from the initial position; negative positions are anticlockwise.
//there is no feedback of the motor's real position. We only know where we _think_ it is, relative to the initial starting point

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(3000);
  muovi(10,5,Vmax,AccMax);
  delay(3000);
  muovi(10,5,Vmax,AccMax);
}

//muovi viene invocata per muovere gli stepper in syncro
void muovi(long numStepSx,bool dirSx,long numStepDx,bool dirDx,long vel,long acc){
  //trovo i 6 intervalli

  //e' il doppio della rampa per vedere se
  long sSX12=
  long sDX12=
}
//mock del movimento degli stepper
void oneStep(bool dir,bool sxdx){
  if(sxdx)
    Serial.println("passo sinistro");
  else
    Serial.println("passo sinistro");
}


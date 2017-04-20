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
  
}
//mock del movimento degli stepper
void oneStep(bool dir,bool sxdx){
  if(sxdx)
    Serial.println("passo sinistro");
  else
    Serial.println("passo sinistro");
}


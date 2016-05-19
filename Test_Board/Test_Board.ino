unsigned long prevmil[1]={0};
unsigned long mil[1]={0};
bool state=false;

void setup() {
  Serial.begin(9600);
  Serial.println();
  pinMode(13,OUTPUT);               //Indicator light
}

void loop() {
  if (millis()-prevmil[0]>=100){
    digitalWrite(13,state);
    state=!state;
    prevmil[0]=millis();
  }
}

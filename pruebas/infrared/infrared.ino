#define IRledPin 3
#define IRsensorPin 4
#define D13ledPin 2    //led que indica visualmente la deteccion del obstaculo

void IR38Write() {
  for(int i = 0; i <= 384; i++) {
    digitalWrite(IRledPin, HIGH);
    delayMicroseconds(13);
    digitalWrite(IRledPin, LOW);
    delayMicroseconds(13);
  }
}

void setup(){
  pinMode(IRledPin, OUTPUT);
  digitalWrite(IRledPin, LOW);
  pinMode(D13ledPin, OUTPUT);
  digitalWrite(D13ledPin, LOW);
}
 
 
void loop(){

  IR38Write();
  if (digitalRead(IRsensorPin)==LOW){
    digitalWrite(D13ledPin, HIGH);
  } else {
    digitalWrite(D13ledPin, LOW);
  }
  delay(100);

//  digitalWrite(D13ledPin, HIGH);
//  delay(80000);
}
 


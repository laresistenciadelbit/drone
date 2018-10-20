int PIN=9; //PWM

void setup()
{
   //Serial.begin(9600); 
   pinMode(PIN, OUTPUT); 
}

void loop()
{
  analogWrite(PIN, 0);
  delay(4000);
  analogWrite(PIN, 216);
  delay(4000);   
  analogWrite(PIN, 0);
  delay(4000);  
  analogWrite(PIN, 223);
  delay(4000);  
   analogWrite(PIN, 0);
  delay(4000);  
  analogWrite(PIN, 231);
  delay(4000);  
  analogWrite(PIN, 0);
  delay(4000);  
  //analogWrite(PIN, 120);
  //delay(4000);  
  
  
   // analogWrite(PIN, 0);
  delay(200000);  


}


int PIN=5; //PWM

void setup()
{
   //Serial.begin(9600); 
   pinMode(PIN, OUTPUT); 
}

void loop()
{
  analogWrite(PIN, 0);
  delay(4000);
  analogWrite(PIN, 20);
  delay(4000);   
  analogWrite(PIN, 0);
  delay(4000);  
  analogWrite(PIN, 40);
  delay(4000);  
   analogWrite(PIN, 0);
  delay(4000);  
  analogWrite(PIN, 70);
  delay(4000);  
  analogWrite(PIN, 0);
  delay(4000);  
  //analogWrite(PIN, 120);
  delay(4000);  
  
  
    analogWrite(PIN, 0);
  delay(300000);  


}


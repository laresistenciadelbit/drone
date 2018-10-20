#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

RF24 radio(7,8);

const uint64_t pipe = 0xE8E8F0F0E1LL;

int time;

void setup(void)
{
   Serial.begin(9600);
   radio.begin();
   radio.openReadingPipe(1,pipe);
   radio.printDetails();
   radio.startListening();
}

void loop(void)
{
   //Serial.print("Number received: ");
   //radio.read( &time, sizeof(int) );
   //Serial.println(time);
   //delay(200);
   
  if ( radio.available() )
  {
    // Dump the payloads until we've gotten everything
     radio.read( &time, sizeof(int) );
     Serial.println(time);
  }
  else
  {
    Serial.println("No radio available");
  }
  delay(400);
}

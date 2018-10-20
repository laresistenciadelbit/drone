#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

RF24 radio(7,8);

const uint64_t pipe = 0xE8E8F0F0E1LL;

void setup(void)
{
   Serial.begin(9600);
   radio.begin();
   radio.openWritingPipe(pipe);
   radio.printDetails();
}

void loop(void)
{
   int time = 3;
   Serial.print("Sending number: ");
   Serial.println(time);
   radio.write( &time, sizeof(int) );
   delay(400);
}

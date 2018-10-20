/**
 * Pins:
 * MISO -> 12
 * MOSI -> 11
 * SCK -> 13
 * Note: To see best case latency comment out all Serial.println
 * statements not displaying the result and load 
 * 'ping_server_interupt' on the server.
 */

#include <SPI.h>
#include <Mirf.h>
#include <Mirf_nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

void setup(){
  /*Mirf config*/
	Mirf.cePin = 7;
	Mirf.csnPin = 8;
	Mirf.spi = &MirfHardwareSpi;
	Mirf.init();
   	Mirf.setRADDR((byte *)"clie1");
	Mirf.payload = sizeof(unsigned long);
    Mirf.channel = 19; //
	Mirf.configRegister( RF_SETUP, ( 1<<2 | 1<<1 ) ); //+power
	Mirf.config();	
  /*Mirf config END*/
  
  Serial.begin(9600);
  Serial.println("Beginning ... "); 
}

void loop(){
  unsigned long time = millis();
  
  Mirf.setTADDR((byte *)"serv1");
  
  Mirf.send((byte *)&time);
  
  while(Mirf.isSending()){
  }
  Serial.println("Finished sending");
  delay(10);
  while(!Mirf.dataReady()){
    //Serial.println("Waiting");
    if ( ( millis() - time ) > 1000 ) {
      Serial.println("Timeout on response from server!");
      return;
    }
  }
  
  Mirf.getData((byte *) &time);
  
  Serial.print("Ping: ");
  Serial.println((millis() - time));
  
  delay(1000);
} 
  
  
  

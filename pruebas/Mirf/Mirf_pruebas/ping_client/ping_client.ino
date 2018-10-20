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
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

struct paquetes {
   unsigned long  time;
   int c=0; //command
   int s=0; //subcommand
} paquete;

void setup(){
  /*Mirf config*/
	Mirf.cePin = 7;
	Mirf.csnPin = 8;
	Mirf.spi = &MirfHardwareSpi;
	Mirf.init();
   	Mirf.setRADDR((byte *)"clie1");
	Mirf.payload = sizeof(paquetes);
    Mirf.channel = 19; //
	Mirf.configRegister( RF_SETUP, ( 1<<2 | 1<<1 ) ); //+potencia
	Mirf.config();	
  /*Mirf config END*/
  
  Serial.begin(9600);
  Serial.println("Beginning ... "); 
}

void loop(){
  paquete.c++;
  paquete.s++;paquete.s++;
  paquete.time = millis();
  
  Mirf.setTADDR((byte *)"srv1");
  
  Mirf.send((byte *)&paquete);
  
  while(Mirf.isSending()){
  }
  Serial.println("Finished sending");
  delay(10);
  while(!Mirf.dataReady()){
    //Serial.println("Waiting");
    if ( ( millis() - paquete.time ) > 1000 ) {
      Serial.println("Timeout on response from server!");
      return;
    }
  }
  
  Mirf.getData((byte *) &paquete.time);
  
  Serial.print("Ping: ");
  Serial.println((millis() - paquete.time));
  
  delay(3000);
} 
  
  
  

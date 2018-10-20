/**
 * Pins:
 * MISO -> 12
 * MOSI -> 11
 * SCK -> 13
 */

#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

struct paquetes {
   unsigned long  time;
   int c; //command
   int s; //subcommand
} paquete;

void setup(){
  Serial.begin(9600);
  
  Mirf.cePin = 4;//Mirf.cePin = 7; <-4 for pro micro (7will be using interruption for acc)
  Mirf.csnPin = 8;
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((byte *)"srv1");
  Mirf.payload = sizeof(paquetes);
  Mirf.channel = 19; //
  Mirf.configRegister( RF_SETUP, ( 1<<2 | 1<<1 ) ); //+power
  Mirf.config();
  delay(8000);
  Serial.println("Listening..."); 
}

void loop(){
   
  //byte data[Mirf.payload];
   
  if(!Mirf.isSending() && Mirf.dataReady()){
     Serial.println("Got packet");
    Mirf.getData((byte *) &paquete);  //coge paquete del buffer
     Serial.print(paquete.c);Serial.print(" - ");Serial.print(paquete.s);Serial.print("\n");
    Mirf.setTADDR((byte *)"clie1");    //responde
    Mirf.send((byte *) &paquete.time);//devuelve solo el tiempo
     Serial.println("Reply sent.");
  }
}

#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

  unsigned int numero = 0;
  unsigned int numero_recibido;
  
  
void setup(){
  Serial.begin(9600);
  //Mirf.cePin = 8;
  //Mirf.csnPin = 7;
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((byte *)"cliente");

  Mirf.payload = sizeof(unsigned int);  //tipo de dato
  //* To change channel: (0-127)
        Mirf.channel = 19;

  Mirf.config();
  Serial.println("Beginning ... "); 
}


void loop(){
  numero++;
    
  Mirf.setTADDR((byte *)"servidor");
  Mirf.send((byte *)&numero);
  while(Mirf.isSending()){}
  
  Serial.print("Finished sending ");
  Serial.print(numero);
  delay(10);
   
  if(!Mirf.dataReady()){
  Serial.println("NOT READY");
  delay(30);
}
  //Serial.println("Waiting");
  Mirf.getData((byte *) &numero_recibido);
  Serial.print("recibido: ");
  Serial.println(numero_recibido);
  
  delay(1000);
} 
  
  
  

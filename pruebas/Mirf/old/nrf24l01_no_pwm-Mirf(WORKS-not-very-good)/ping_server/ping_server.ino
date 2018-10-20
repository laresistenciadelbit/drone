#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

void setup(){
  Serial.begin(9600);
  Mirf.cePin = 8;//
  Mirf.csnPin = 7;//
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((byte *)"servidor");   
  Mirf.payload = sizeof(unsigned int);
   Mirf.channel = 19;
  Mirf.config();
  
  Serial.println("Listening..."); 
}

void loop(){
  unsigned int numero_recibido;
  byte data[Mirf.payload];
    
  if(!Mirf.isSending() && Mirf.dataReady()){
    //get the packet
    Mirf.getData((byte *) &numero_recibido); //Mirf.getData(data);
    Serial.println("Got packet!");
    Serial.println(numero_recibido);
     
    delay(10);
    //Send the data back to the client.
    Mirf.setTADDR((byte *)"cliente");
    Mirf.send((byte *)&numero_recibido); //Mirf.send(data); 
    while(Mirf.isSending()){ }  
    Serial.println("Reply sent.");
  }
}

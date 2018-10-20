/**
 * Pins:
 * MISO -> 12
 * MOSI -> 11
 * SCK -> 13
 */

#include <SPI.h>
#include <Mirf.h>
#include <Mirf_nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

void setup(){
  Serial.begin(9600);
  
  Mirf.cePin = 7;
  Mirf.csnPin = 8;
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((byte *)"serv1");
  Mirf.payload = sizeof(unsigned long);
  Mirf.channel = 19; //
  Mirf.configRegister( RF_SETUP, ( 1<<2 | 1<<1 ) ); //+power
  Mirf.config();
  
  Serial.println("Listening..."); 
}

void loop(){
  /*
   * A buffer to store the data.
   */
   
  byte data[Mirf.payload];
  
  /*
   * If a packet has been recived.
   *
   * isSending also restores listening mode when it 
   * transitions from true to false.
   */
   
  if(!Mirf.isSending() && Mirf.dataReady()){
    Serial.println("Got packet");
    
    /*
     * Get load the packet into the buffer.
     */
     
    Mirf.getData(data);
    
    /*
     * Set the send address.
     */
     
     
    Mirf.setTADDR((byte *)"clie1");
    
    /*
     * Send the data back to the client.
     */
     
    Mirf.send(data);
    
    /*
     * Wait untill sending has finished
     *
     * NB: isSending returns the chip to receving after returning true.
     */
      
    Serial.println("Reply sent.");
  }
}

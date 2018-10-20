#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

/*ledsnotworking*///#include <SimpleTimer.h>  //simpletimer	//lo usaremos para encender los leds del mando si se queda sin bateria (él o el dron) o si no recibe respuesta del dron
/*ledsnotworking*///SimpleTimer timer;        //simpletimer

#define XBOX_USB          //para controlador xbox usb
//#define XBOX_WIRELESS   //para controlador xbox wireless

#ifdef XBOX_USB
  #include <XBOXUSB.h>	
#else
  #include <XBOXRECV.h>
#endif
USB Usb;

#ifdef XBOX_USB
  XBOXUSB Xbox(&Usb);
#else
  XBOXRECV Xbox(&Usb);
#endif

boolean debug=true;

long int leftStickX,leftStickY,rightStickX,rightStickY;

void send_to_quad(int,int);

/*						comando
pad1_x: roll		-> 1
pad1_y: pitch		-> 2
l2,r2: power_all	-> 3
l1,r1: yaw(giro sobre si mismo) � eje x de pad2	 -> 4
pad2_x: motores en eje x -> 5
pad2_y: motores en eje x -> 6
x -> 7
y -> 8
a -> 9
b -> 10
select -> 11
l3 -> 12
r3 -> 13
up -> 14   : avanza 1m hacia adelante
down -> 15 : // hacia atrás
left -> 16 : hacia izda
right -> 17

r1+x -> 4.10
r1+y -> 4.11
r1+a -> 4.12
r1+b -> 4.13

start: arranca o para motores -> 100
*/
 //PINS:
 // MISO -> 12
 // MOSI -> 11
 // SCK -> 13

void setup()
{
/*Mirf config*/
	Mirf.cePin = 7;
	Mirf.csnPin = 8;
	Mirf.spi = &MirfHardwareSpi;
	Mirf.init();
   	Mirf.setRADDR((byte *)"clie1");
	Mirf.payload = sizeof(unsigned long);
    Mirf.channel = 19;
	Mirf.configRegister( RF_SETUP, ( 1<<2 | 1<<1 ) );
	Mirf.config();	
/*Mirf config END*/

	if(debug)Serial.begin(9600); //OPT
      // Halt program until shield is connected  // USB.Init will return -1 for shield disconnected and 0 if connected
  if (Usb.Init() == -1)    // If USB shield did not initialise for whatever reason...
  {
	if(debug)Serial.println("Usb shield disconnected");
    while(Usb.Init() == -1) delay(3000); //halt as long as shield is reported disconnected
  }
  
/*ledsnotworking*///  timer.setTimeout(3000, OnceOnlyTask );  //simpletimer //apaga los leds tras 4 segundos de ser parpadeados por algún fallo (batería, señal...)
}

/*ledsnotworking*///void OnceOnlyTask() {Xbox.setLedOff();}

struct paquetes {
   unsigned long  time;
   int c=0; //command
   int s=0; //subcommand
} paquete;

int min_limit_stick=15; //velocidad minima (sensibilidad) para que el stick lo mande al dron (0-255).

void loop()
{
	Usb.Task();
	if(Xbox.Xbox360Connected) 
	{
		leftStickX = map(Xbox.getAnalogHat(LeftHatX), -32768, 32767, -255, 255);  
		if(leftStickX > min_limit_stick || leftStickX < -min_limit_stick)
			send_to_quad(1,leftStickX); //(comando 1 (roll), velocidad)
		  
		leftStickY = map(Xbox.getAnalogHat(LeftHatY), -32768, 32767, -255, 255);  
		if(leftStickY > min_limit_stick || leftStickY < -min_limit_stick)
			send_to_quad(2,leftStickY); //(comando 2 (pitch), velocidad)


    rightStickX = map(Xbox.getAnalogHat(RightHatX), -32768, 32767, -255, 255);  
    if(rightStickX > min_limit_stick || rightStickX < -min_limit_stick)
      send_to_quad(5,-rightStickX); //(comando 5 [reduce la velocidad del motor para ir en esa direccion (eje_x)], velocidad)
      
    rightStickY = map(Xbox.getAnalogHat(RightHatY), -32768, 32767, -255, 255);  
    if(rightStickY > min_limit_stick || rightStickY < -min_limit_stick)
      send_to_quad(6,-rightStickY); //(comando 6 [reduce la velocidad del motor para ir en esa direccion (eje_y)], velocidad)


		//if ( Xbox.getButtonClick(L2) ) //power_all_-		//Mapeamos de 0-255 a 0-127 //<-si usásemos getbuttonclick solo lo coje una sola vez
    if ( Xbox.getButtonPress(L2)>3 ) //power_all_-   //Mapeamos de 0-255 a 0-127  //ponemos de mínimo 3, podíamos haber puesto 1 o 0 pero así evitamos errores
		{
			send_to_quad(3, ( 127 - map( Xbox.getButtonPress(L2),0,255,0,127 ) ) );
		}
		if ( Xbox.getButtonPress(R2)>3 ) //power_all_+		//Mapeamos de 0-255 a 128-255
		{
			send_to_quad(3, ( map( Xbox.getButtonPress(R2),0,255,0,127 ) + 128 )  );
		}

		if ( Xbox.getButtonClick(L1) ) //yaw_left
		{
			send_to_quad(4,0);
		}
		if ( Xbox.getButtonClick(R1) ) //yaw_right
		{
			send_to_quad(4,1);
		}
		
		if ( Xbox.getButtonClick(R1) ) //dos botones pulsados a la vez (r1 + x,b,y,a) <-funcion de vuelo preprogramada
		{
			if ( Xbox.getButtonClick(X) )
				send_to_quad(4,10);
			if ( Xbox.getButtonClick(Y) )
				send_to_quad(4,11);
			if ( Xbox.getButtonClick(A) )
				send_to_quad(4,12);
			if ( Xbox.getButtonClick(B) )
				send_to_quad(4,13);
			
		}

		if ( Xbox.getButtonClick(START) ) //arranca motores y estabiliza
		{
			send_to_quad(100,100);
		}	
		if ( Xbox.getButtonClick(BACK) ) //(select)
		{
			send_to_quad(11,11);
		}

		if (Xbox.getButtonClick(X))     send_to_quad(7,7);
		if (Xbox.getButtonClick(Y))     send_to_quad(8,8);
		if (Xbox.getButtonClick(A))     send_to_quad(9,9);
		if (Xbox.getButtonClick(B))     send_to_quad(10,10);

    if (Xbox.getButtonClick(L3))    send_to_quad(12,12);
    if (Xbox.getButtonClick(R3))    send_to_quad(13,13);
	if (Xbox.getButtonClick(UP))    send_to_quad(14,14);	//avanza 1m
	if (Xbox.getButtonClick(DOWN))  send_to_quad(15,15);
	if (Xbox.getButtonClick(LEFT))  send_to_quad(16,16);
	if (Xbox.getButtonClick(RIGHT)) send_to_quad(17,17);
	}
}


void send_to_quad(int c, int s)
{
	paquete.c=c;
	paquete.s=s;
	paquete.time = millis();
  
	Mirf.setTADDR((byte *)"srv1");
	Mirf.send((byte *)&paquete);
  
  	if(debug)
	{
		Serial.print("Sending: ");
		Serial.print("[ ");
		Serial.print(c);
		Serial.print(" ][ ");
		Serial.print(s);
		Serial.print(" ]\n");
	}

	while(Mirf.isSending()){  }
	if(debug)Serial.println("Finished sending");
	delay(10);
	while(!Mirf.dataReady())
	{
		if ( ( millis() - paquete.time ) > 1000 ) //si no recibimos respuesta en 1 segundo
		{		//si el dron está estabilizándose, no se si pierde el paquete o le llega con retraso, en cualquier caso no se ha de tener en cuenta.
			if(debug)Serial.println("Timeout on response from server!");
		  
			/*ledsnotworking*///  	  Xbox.setLedOn(LED1);//encendemos led 2 [paquete no contestado] (setLedBlink o setLedOn)
			/*ledsnotworking*///  	  //timer.run();//apagamos los leds a los 3 segundos
			
			return;	//<---- SALE directamente de la función, ya que no hemos recibido respuesta en 1 segundo
		}
	}

	Mirf.getData((byte *) &paquete.time);	//el return de antes evita esto si no hemos recibido nada

	if(debug)Serial.print("Ping: ");
	if(debug)Serial.println((millis() - paquete.time));
}

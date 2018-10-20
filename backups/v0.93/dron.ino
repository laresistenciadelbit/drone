// 0 .__. 1    <--motors
//   |  |
// 2 .__. 3 
// * <- significa que falta codigo

#define USE_HC-SR04 //si usamos sensor de distancia
/*#define USE_MS5611*/		//si usamos barometro MS5611
#define USE_RF24 //si usamos nRF24L01 con mirf

const boolean debug=false;
const int min_speed=80; //0-255 velocidad mínima de movimiento de las aspas (para tener en cuenta al aterrizar)

#include <SimpleTimer.h> //será nuestra libreria de Threads (timers por software que se ejecutan paralelamente),no es una interrupción porque no para la ejecución actual, pero si se ejecuta paralelamente.

#include <Wire.h>	//para acc y ms5611

 //Acelerometro
#include "I2Cdev.h" //para usar el acelerometro
#include "MPU6050_6Axis_MotionApps20.h" //usamos DMP tambien
//
MPU6050 accelgyro;
//int16_t ax, ay, az;
//int16_t gx, gy, gz;
#include "dmp_vars.h"

 //Mirf
#ifdef USE_RF24
	#include <SPI.h>
	#include <Mirf.h>
	#include <Mirf_nRF24L01.h>
	#include <MirfHardwareSpiDriver.h>
#endif

#ifdef  USE_HC-SR04
  #include "F_HC-SR04.cpp" //mis funciones
#endif

#ifdef	USE_MS5611
	#include <MS5611.h>
	MS5611 ms5611;
	double referencePressure;
  #include "F_MS5611.cpp" //mis funciones
#endif

const int motor_pin[4]={5,6,9,10}; //pines de los motores //meter en setup?
int speed[4]={0,0,0,0};//vector de velocidades
int stable[4]={200,200,200,200};// 0-255 vector de velocidad estable (se reiniciaran solos los valores)
int x,y,z=0/*,gx,gy,gz*/, x_aux=0,y_aux=0,z_aux=0; //para no tener que crearlas cada vez que entra en get_position // x e y son grados, z es aceleracion (ya no se usa gy,gx y todo es del dmp)

const int starting_speed=220; //0-255 / es un valor sin medir, luego se estabilizara el solo.
const int regulable_speed=10; //suma o resta velocidad en tramos de regulable_speed (10) para calibrar z (que sea divisible entre 2)

const float Xoffset=0,Yoffset=2.75; //calibrado a ojo (a veces da 2.75, a veces 3.5)
const int stable_z=42;//altura de estabilización la ponemos a 42cm. (para tener un margen entre 40 y 44)
const int Zmargin=2;
const int stop_z=20; //a partir de esta altura (20cm) se reducira durante 1 segundo la velocidad de los motores y se parara.
//const int no_stable_z=2000; ya no se usa (era para acc en eje z)
//const int stablishing_z_limit=200; //velocidad limite de subida para autoestabilizacion
const int no_stable_xy_strict=8;  //por debajo o encima de este numero el eje x o y no esta estabilizado (mientras no se mueve)
const int no_stable_xy_limit=45; //estabilizacion menos estricta (durante el vuelo)
const boolean AUTO_BOOT_WHEN_UNSTABLE=true; //si se activa, al arrancar si no recive comandos, cada medio segundo comprueba si esta estable, sino, arranca

int *get_width(); //funcion puntero que contendrá HC-SR04(ultrasonidos) o MS5611(presion atm) según cual de los dos se use

#include "header.h"

unsigned long time, last_t;
void sleep(unsigned int delay_t)
{
	time=millis();
	last_t=time;
	while( last_t<(time+delay_t) )
		last_t=millis();
}
/*Sacamos la altura a partir del acelerometro (de momento INVIABLE)
	int stabilize_z()	//usamos giroscopio (ahora usamos cambios de gyro en x e y si los angulos son estables) +/-2000
	{
		//do{
			z=get_z_acc();
			
			if(z < -no_stable_z)
				power_all(+regulable_speed,false);
			else if(z > no_stable_z) 
				power_all(-regulable_speed,false);
		//}while(z< -no_stable_z || z>no_stable_z); //valor a partir del cual la altura no es estable
	}

	int get_z_acc()
	{
		while (!mpuInterrupt && fifoCount < packetSize) {}
		mpuInterrupt = false;
		mpuIntStatus = accelgyro.getIntStatus();
		fifoCount = accelgyro.getFIFOCount();
		if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
			accelgyro.resetFIFO();
		} else if (mpuIntStatus & 0x02) {
			while (fifoCount < packetSize) fifoCount = accelgyro.getFIFOCount();
			accelgyro.getFIFOBytes(fifoBuffer, packetSize);
			fifoCount -= packetSize;
				accelgyro.dmpGetQuaternion(&q, fifoBuffer);
				accelgyro.dmpGetGravity(&gravity, &q);
				accelgyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
				accelgyro.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
		}
		return(aaWorld.x);
	}*/
	

void get_angle()	// 10ms por cada lectura
{
    while (!mpuInterrupt && fifoCount < packetSize) {}
    mpuInterrupt = false;
    mpuIntStatus = accelgyro.getIntStatus();
    fifoCount = accelgyro.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        accelgyro.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = accelgyro.getFIFOCount();
        accelgyro.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
            accelgyro.dmpGetQuaternion(&q, fifoBuffer);
            accelgyro.dmpGetGravity(&gravity, &q);
            accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
            x = ypr[0] * 180/M_PI;
            y = ypr[1] * 180/M_PI;
            //z = ypr[2] * 180/M_PI; //quitamos z ya que no nos interesa la informacion de los grados en ese eje para la estabilizacion (eje de ROLL)
    }
}

boolean is_stable_z()
{
	int medidas=3;
	int width=*get_width(); //Medida inicial
	//ver cuantas medidas puede tomar en 0.2 segundos
	//de momento usamos 3 (contando la inicial)
	for(int i=0; i<medidas-1; i++)
		width+=*get_width();
	width=width/medidas;
	
	return(  width < stable_z -Zmargin || width > stable_z +Zmargin )
}

boolean is_stable_xy()
{
	//z=get_z_acc();
	get_angle();
	return (y>Yoffset+no_stable_xy_limit || y<Yoffset-no_stable_xy_limit || x>Xoffset+no_stable_xy_limit || x<Xoffset-no_stable_xy_limit ) //aumentamos el limite +-100 en eje z
}

boolean is_stable(){
return(is_stable_xy && is_stable_z);
}


void stabilize_z()
{
	//do{
		z=*get_width();
		
		if(z < stable_z -Zmargin )
			power_all(+regulable_speed,false);
		else if(z > stable_z +Zmargin) 
			power_all(-regulable_speed,false);
	//}while( z< (-stable_z -Zmargin) || z>(stable_z +Zmargin) ); //valor a partir del cual la altura no es estable
}

void stabilize_xy(int no_stable_xy=45)
{
	//do{
		get_angle();
		if(y>Yoffset+no_stable_xy || y<Yoffset-no_stable_xy) pitch(regulable_speed,false);
		if(x>Xoffset+no_stable_xy || x<Xoffset-no_stable_xy)  roll(regulable_speed,false);
	//}while(y>Yoffset+no_stable_xy || y<Yoffset-no_stable_xy || x>Xoffset+no_stable_xy || x<Xoffset-no_stable_xy);
}

void stabilize_xyz(int no_stable_xy=45)
{
	do{
		stabilize_xy(no_stable_xy);
		stabilize_z();
	}while( (y>Yoffset+no_stable_xy) || (y<Yoffset-no_stable_xy) || (x>Xoffset+no_stable_xy) || (x<Xoffset-no_stable_xy) || (z< -no_stable_z) || (z>no_stable_z) );
	
	for(int i=0;i<3;i++)			//poner este valor cada cuanto o cuando? cuando esté muy estable? tener en cuenta
		stable[i]=speed[i];
}

void power(int motor_number)
{ //put speed to a motor
	if(speed[motor_number]>255)		//we verify that it doesnt over speed it
		speed[motor_number]=255;
		
	analogWrite(motor_pin[motor_number], speed[motor_number]);
}
void power_all(int add, boolean abs=true)	//abs->absolute, if true put speed, if false add speed.
{ //add speed to all motors				//we verify that it doesnt over speed it
	for(int i=0;i<4;i++){
		if(abs) speed[i]=add;
		else	speed[i]+=add;
		power(i);
	}
}

void stop()
{
	stabilize_xyz(no_stable_xy_strict); //estabilizamos primero
	sleep(100);	//esperamos por si los valores de z no son estables aun
	do{
		z=*get_width();
		//z=get_z_acc();
		if(speed[0]+speed[1]+speed[2]+speed[3]>min_speed) //al descender no bajamos de 80 de velocidad (sino las aspas se pararían)
			power_all(-regulable_speed,false);
		sleep(500);
	}while(z>stop_z || speed[0]+speed[1]+speed[2]+speed[3]<min_speed);	//si no hemos mapeado, por debajo de 80 probablemente no se muevan los motores. Asi que se sale si los motores se han parado ya y el acc no ha medido bien, o no ha llegado al suelo y se ha parado.
}

void hard_stop() //parada completa de motores, ----esta y stop se pueden llamar en cualquier momento **********
{
		power_all(0,true);
		//wait for start command
}

struct paquetes {
   unsigned long  time;
   int c; //command
   int s; //subcommand
} paquete;

boolean get_command(int *p1, int *p2) //recive los comandos por el modulo inalambrico.
{
	boolean REPLY=false;
	
#ifdef USE_RF24	//si usamos nRF24L01 con Mirf
	byte data[Mirf.payload];
   
	if(!Mirf.isSending() && Mirf.dataReady()){
		 if(debug)Serial.println("Got packet");
		Mirf.getData((byte *) &paquete);  //coge paquete del buffer
		 if(debug)Serial.println(paquete.c);
		 if(debug)Serial.println(paquete.s);
		Mirf.setTADDR((byte *)"cli1");    //responde
		Mirf.send((byte *) &paquete.time);//devuelve solo el tiempo
		 if(debug)Serial.println("Reply sent.");
	}
#endif
	
	*p1=paquete.c;
	*p2=paquete.s;
	
	if (REPLY) return(true);
	else return(false);
}

void roll(int add, boolean abs=true) //movimiento del eje x 		//abs->absolute, if true put speed, if false add speed.
{
	if(abs)
	{
	 speed[1]=stable[1]-(add/2);	//velocidad estable - velocidad añadida/2 en motor 1
	 speed[3]=stable[3]+(add/2);
	}else{
	 speed[1]-=(add/2);	//escribimos la nueva velocidad en el vector
	 speed[3]+=(add/2);
	}
	power(1); //quitamos mitad de velocidad de la pulsada al motor 3 (para valores positivos)
	power(3); //damos mitad de velocidad de la pulsada al motor 1    (para valores positivos)
}

void pitch(int add, boolean abs=true)  //movimiento del eje y		//abs->absolute, if true put speed, if false add speed.
{
	if(abs)
	{
	 speed[0]=stable[0]-(add/2);
	 speed[2]=stable[2]+(add/2);
	}else{
	 speed[0]-=(add/2);
	 speed[2]+=(add/2);
	}
	power(0);
	power(2);
}

void yaw(int add) //movimiento de giro sobre si mismo (z==gravedad) (usar l1,r1)
{
	 speed[0]=stable[0]-(add/2);
	 speed[2]=stable[2]-(add/2);
	 speed[1]=stable[1]+(add/2);
	 speed[3]=stable[3]+(add/2);
	power(0);
	power(2);
	power(1);
	power(3);
}

void motor_x(int add) //reduce la velocidad del eje_x del dron para moverse hacia ese lado
{
	 speed[0]=stable[0]-(add/2);
	 speed[3]=stable[3]-(add/2);
	 speed[1]=stable[1]+(add/2);
	 speed[2]=stable[2]+(add/2);
	power(0);
	power(3);
	power(1);
	power(2);
}
void motor_y(int add) //reduce la velocidad del eje_y del dron para moverse hacia ese lado
{
	 speed[0]=stable[0]-(add/2);
	 speed[1]=stable[1]-(add/2);
	 speed[2]=stable[2]+(add/2);
	 speed[3]=stable[3]+(add/2);
	power(0);
	power(1);
	power(2);
	power(3);
}

void setup()
{
	for(int i=0; i<4; i++) pinMode(motor_pin[i], OUTPUT);
	
	#ifdef	USE_MS5611	//si usamos barometro para estabilizar eje z
		if(debug)
			while(!ms5611.begin())
				Serial.println("Could not find a valid MS5611 sensor, check wiring!");
		referencePressure = ms5611.readPressure();
		if(debug)	Serial.println(ms5611.getOversampling());
		
		get_width=F_HC-SR04;	//metemos la funcion en una funcion puntero
	#endif
	
	#ifdef	USE_HC-SR04	//si usamos sensor de ultrasonidos para estabilizar eje z
		pinMode(trigPin, OUTPUT); //trigPin (18 is a0 in proMicro)
		pinMode(echoPin, INPUT);  //echoPin (19 is a1 in proMicro)
		
		get_width=F_MS5611;	//metemos la funcion en una funcion puntero
	#endif
	
	//{Acc  config BEGIN}
		#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
			Wire.begin();
			TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
		#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
			Fastwire::setup(400, true);
		#endif
		
		accelgyro.initialize();
		devStatus = accelgyro.dmpInitialize();
		//define offests here (if you want)
		accelgyro.setDMPEnabled(true);
		attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = accelgyro.getIntStatus();
		dmpReady = true;
		packetSize = accelgyro.dmpGetFIFOPacketSize();
	//{Acc  config END}
	//{Mirf config BEGIN}
#ifdef USE_RF24
		Mirf.cePin = 7;
		Mirf.csnPin = 8;
		Mirf.spi = &MirfHardwareSpi;
		Mirf.init();
		Mirf.setRADDR((byte *)"srv1");
		Mirf.payload = sizeof(paquetes);
		Mirf.channel = 19; //
		Mirf.configRegister( RF_SETUP, ( 1<<2 | 1<<1 ) ); //+power
		Mirf.config();
#endif		
	//{Mirf config END}
	
	if(debug)
	{
		Serial.begin(115200);
		Serial.println("Testing device connections...");
		Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
	}
}

void loop()
{
	unsigned long time_start,time_diff; boolean time_started=false;//time1
    if (!dmpReady) {
      if(debug) Serial.println("Accelerator failed");
      return; //(acc) if programming failed, don't try to do anything
    }
	//boolean use_acc=true;
	//boolean use_gyro=false;
 int second_counter=0;
	boolean REPLY=false;
	int p1=0,p2=0; //parametros recibidos del modulo inalambrico (motor/speed)

	//*low battery indicator
  
	//*         TO DO          *//
	
	if(debug) Serial.println("waiting for press start button");
	while(p1!=100)	//mientras no se haya mandado la orden start
	{
		if(AUTO_BOOT_WHEN_UNSTABLE)
		{
			if(!time_started){	//contamos tiempo para estabilizar estricta o no estrictamente.
				time_start = millis();			
				time_started=true;				
			}
			time_diff = millis() - time_start;
			if(time_diff>700)
				if(!is_stable()) //si no esta estable arranca
				{
					if(debug){
						Serial.print("Arrancamos por inestabilidad: ");
						Serial.print("grados y = ");  Serial.print(y); Serial.print(" / grados x = ");  Serial.print(x); Serial.print(" / acc z = "); Serial.print(z);
					}
					time_started=false;//paramos el time_started porque lo vamos a reutilizar despues.
					break;
				}
		}
		REPLY=get_command(&p1,&p2);
	}

	//*motors activated
		power_all(starting_speed,true);
		sleep(1000);

	//*stabilising xyz
		stabilize_xyz(no_stable_xy_strict);
	
	//once stabilised wait for orders and stabilize again:
	do{
		REPLY=get_command(&p1,&p2);
		if(REPLY)
		{
			second_counter=0;
			
			switch(p1)	//ordenes que no sean de mover un motor
			{
				case 1: roll(p2);		//(+) se mueve en el eje x
				break;
				case 2: pitch(p2);		//(+) se mueve en el eje y
				break;
				case 3: power_all(p2);	// aumenta o reduce vel. de todos los motores
				break;
				case 4: yaw(p2);		// gira sobre si mismo
				break;
				case 5: motor_x(p2);	//(x) se mueve sobre el eje x
				break;
				case 6: motor_y(p2);	//(x) se mueve sobre el eje y
				break;
				
				default:
				case 100: stop();
				return;
			}
		}
	//stabilize:
		if(!time_started){	//contamos tiempo para estabilizar estricta o no estrictamente.
			time_start = millis();			
			time_started=true;
		}
		time_diff = millis() - time_start;
		
		if(time_diff<1000)	stabilize_xyz();//stabilize dron with {no_stable_xy_limit} 
		else { //If not receive command in 1.5 seconds: stabilize drone with {no_stable_xy_strict}
			stabilize_xyz(no_stable_xy_strict);
			time_started=false;
			time_start = millis();
			for(int i=0;i<4;i++)	stable[i]=speed[i];
			
			second_counter++; //contamos los segundos en los que no se le ha mandado comandos.
		}
		//stabilize_z(); called in stabilize_xyz();
		
		if(second_counter>60) stop();//CONTAR 60 SEGUNDOS Y VOLVER A LA BASE <- de momento baja y se apaga
	}while(p1!=100);
	
	sleep(3000);

}

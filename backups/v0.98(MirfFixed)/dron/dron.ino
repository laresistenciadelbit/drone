//CUIDADO con estabilizar con regulable_speed o con permitir velocidades altas de pitch roll, ya que si son mayores que regulable_speed tendremos problemas a la hora de estabilizar (puede que ni llegue a conseguir estabilizar nada)

// 0 .__. 1    <--motors
//   |  |
// 2 .__. 3 
// * <- significa que falta codigo

#define PROMICRO //si usamos arduino pro micro no usamos la interrupcion 0 para el mpu6050 ya que es el pin2 que es sda, así que usamos interrupcion 2 que es pin 0 (rx)
/*#define USE_HCSR04*/ //si usamos sensor de distancia
#define USE_MS5611		//si usamos barometro MS5611  (error de ~20cms)
#define USE_RF24 //si usamos nRF24L01 con mirf

const boolean debug=true;


 //Acelerometro
#include <I2Cdev.h> //para usar el acelerometro
#include <MPU6050_6Axis_MotionApps20.h> //usamos DMP tambien

//para acc y ms5611
#include <Wire.h>

 //Más de acelerometro
MPU6050 accelgyro; //int16_t ax, ay, az;//int16_t gx, gy, gz;
#include "dmp_vars.h"


 //Mirf
#ifdef USE_RF24
	#include <SPI.h>
	#include <Mirf.h>
	#include <nRF24L01.h> //#include <Mirf_nRF24L01.h>
	#include <MirfHardwareSpiDriver.h>
#endif

#ifdef  USE_HCSR04
  int trigPin=18; //trigPin (18 is a0 in proMicro)
  int echoPin=19; //echoPin (19 is a1 in proMicro)
  int F_HCSR04()
  {
    long duration, distance;
    digitalWrite(trigPin, LOW);  // Added this line
    delayMicroseconds(2); // Added this line
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10); // Added this line
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = (duration/2) / 29.1;
    
    if (distance >= 200 || distance <= 0) //solo medimos en distancias menores a 2m
      distance=1989;
    
    return((int)distance);
  }
#endif

#ifdef	USE_MS5611
	#include <MS5611.h>
	MS5611 ms5611;
	double referencePressure;
  int F_MS5611()
  {
    long realPressure = ms5611.readPressure();
    float relativeAltitude = ms5611.getAltitude(realPressure, referencePressure);
    return((int)relativeAltitude*10); //devuelve centimetros
  }
#endif

const int yaw_speed=128; //speed of yaw for giving 90º turn (se necesita probar)
const int yaw_min_speed=40; //speed of yaw for giving 90º turn (se necesita probar)
unsigned int halfsecondcounter=0;
const int min_speed=80; //0-255 velocidad mínima de movimiento de las aspas (para tener en cuenta al aterrizar)
const int lost_height_to_stabilize=70;//70cm perdidos antes de nueva estabilización de altura.
const int ms_to_strict_stabilize=1000; //1s antes de estabilización estricta <----second_counter depende de esta variable (cuidado si la cambiamos)

const int motor_pin[4]={5,6,9,10}; //pines de los motores
int speed[4]={0,0,0,0};//vector de velocidades
int stable[4]={200,200,200,200};// 0-255 vector de velocidad estable (se reiniciaran solos los valores)
int x,y,z=0/*,gx,gy,gz*/, x_aux=0,y_aux=0,z_aux=0; //para no tener que crearlas cada vez que entra en get_position // x e y son grados, z es aceleracion (ya no se usa gy,gx y todo es del dmp)

const int starting_speed=220; //0-255 / es un valor sin medir, luego se estabilizara el solo.
const int regulable_speed=5;//10; //suma o resta velocidad en tramos de regulable_speed (10) para calibrar z (que sea divisible entre 2)
			//\_ej: le metemos sleeps al estabilizar para conseguir en 1000ms un retardo de unos 50 o más ms para que le de tiempo a estabilizarse y haga 1000ms/50ms= 20 estabilizaciones por segundo (puede q sean muchas, pero no conviene hacer sleeps, en todo caso reducir regulable_speed a 5)
const float Xoffset=0,Yoffset=2.75; //calibrado a ojo (a veces da 2.75, a veces 3.5)
const int init_stable_z=100;//altura de estabilización la ponemos a 100cm. (para tener un margen entre 90 y 110cm)
int stable_z;//la vamos a cambiar durante las estabilizaciones (usamos init_stable_z para asignarla al inicio)

const int Zmargin=20/2; //( 20/2 = 10cm por encima y 10cm por debajo de 1m)//*********<---si no funciona bien subir a 25 o 30!!!

const int stop_z=20; //a partir de esta altura (20cm) se reducira durante 1 segundo la velocidad de los motores y se parara.
//const int no_stable_z=2000; ya no se usa (era para acc en eje z)
//const int stablishing_z_limit=200; //velocidad limite de subida para autoestabilizacion
const int no_stable_xy_strict=15;  //por debajo o encima de este numero el eje x o y no esta estabilizado (mientras no se mueve)
const int no_stable_xy_limit=45; //estabilizacion menos estricta (durante el vuelo)
const boolean AUTO_BOOT_WHEN_UNSTABLE=true; //si se activa, al arrancar si no recive comandos, cada medio segundo comprueba si esta estable, sino, arranca
unsigned long time_start,time_diff; boolean time_started=false;//time1
//int *get_width(); 
int (*get_width)(void);//funcion puntero que contendrá HC-SR04(ultrasonidos) o MS5611(presion atm) según cual de los dos se use

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
            //ROLL = ypr[0] * 180/M_PI;	//no necesitamos el angulo de giro
            x = ypr[1] * 180/M_PI;
            y = ypr[2] * 180/M_PI;
    }
}

boolean is_stable_z()	//solo podemos usarla al arrancar ya que tomamos 100cm como stable_z (para estabilizar a 1m de altura)
{//ver cuantas medidas puede tomar en 0.2 segundos	//de momento usamos 3 (contando la inicial)
	int medidas=3;
	z=get_width(); //Medida inicial
	for(int i=0; i<medidas-1; i++) //-1 porque la primera ya la cogimos antes.
		z+=get_width();
	z=z/medidas;
	
	return(  (z > (stable_z -Zmargin) ) && (z < (stable_z +Zmargin) ) );
}

boolean is_stable_xy()
{
	get_angle();
	return (y<(Yoffset+no_stable_xy_strict) && y>(Yoffset-no_stable_xy_strict) && x<(Xoffset+no_stable_xy_strict) && x>(Xoffset-no_stable_xy_strict) );
}

boolean is_stable(){
return(is_stable_xy() && is_stable_z() );
}

void stabilize_z()
{
	//do{
		z=get_width();
		
		if(z < stable_z -Zmargin )
			power_all(+regulable_speed,false);
		else if(z > stable_z +Zmargin) 
			power_all(-regulable_speed,false);
			
		sleep(20);
	//}while( z< (-stable_z -Zmargin) || z>(stable_z +Zmargin) ); //valor a partir del cual la altura no es estable
}

void stabilize_xy(int no_stable_xy=45)
{
	//do{
		get_angle();
		if(y>(Yoffset+no_stable_xy))	pitch(-regulable_speed,false);
		if(y<(Yoffset-no_stable_xy))	pitch(regulable_speed,false);
		if(x>(Xoffset+no_stable_xy))	roll(-regulable_speed,false);
		if(x<(Xoffset-no_stable_xy))  roll(regulable_speed,false);
	//}while(y>Yoffset+no_stable_xy || y<Yoffset-no_stable_xy || x>Xoffset+no_stable_xy || x<Xoffset-no_stable_xy);
}

void stabilize_xyz(int no_stable_xy=45)
{
	do{
		stabilize_xy(no_stable_xy);
		stabilize_z();
		sleep(40); //tiempo de espera para medir si está estabilizado (reducir? aumentar?)
			if(debug)
			{	sleep(200); //damos mas tiempo para no llenar la pantalla de mensajes
				if(!is_stable_xy()){Serial.print("xy NO es estable, x=");Serial.print(x);Serial.print(" / y=");Serial.print(y);Serial.print("\n");}
				if(!is_stable_z()){Serial.print("z NO es estable, z=");Serial.print(z);Serial.print(" cm\n");}
			}
	}while(!is_stable());	//mientras no sea estable xyz.
		//get_angle();	//volvemos a sacar x e y para comparar despues si ya esta estable
	//}while( (y>Yoffset+no_stable_xy) || (y<Yoffset-no_stable_xy) || (x>Xoffset+no_stable_xy) || (x<Xoffset-no_stable_xy) /*|| (z< -no_stable_z) || (z>no_stable_z)*/ );
	
	for(int i=0;i<3;i++)			//añadimos la velocidad estable al array de velocidades estables
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
		z=get_width();
		//z=get_z_acc();
		if(speed[0]+speed[1]+speed[2]+speed[3]>min_speed) //al descender no bajamos de 80 de velocidad (sino las aspas se pararían)
			power_all( -regulable_speed,false);
		sleep(400);
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
	//byte data[Mirf.payload]; <--podíamos haber usado esto para ahorrarnos byte * &paquete, usando data
   
	if(!Mirf.isSending() && Mirf.dataReady()){
		 if(debug)Serial.println("Got packet");
		Mirf.getData((byte *) &paquete);  //coge paquete del buffer
		 if(debug)Serial.println(paquete.c);
		 if(debug)Serial.println(paquete.s);
		Mirf.setTADDR((byte *)"clie1");    //responde
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

void preprogrammed_flight_function(int program)
{
  //LO PODEMOS DESACTIVAR CADA 500ms Y ASI LLEVAMOS LA CUENTA Y PODEMOS USARLO COMO CONTADOR :)
  //O MEJOR, LO DESACTIVAMOS CADA X, DONDE X ES UNA VARIABLE QUE VARÍA EN FUNCION DE LA FUNCION Q QUEREMOS PROGRAMAR
    //ES DECIR, SI ES ESPIRAL, X SERÁ CADA VEZ MAYOR, YA QUE EL RADIO SE VA AGRANDANDO
    
  //if(time_started>940) time_started=false;  //desactivamos contador de estabilizad estricta antes de que se active (por seguridad)
  switch(program)
  {
    case 10:  //espiral con pitch constante y yaw variable con el tiempo cada vez siendo más lenta para hacer una espiral mayor.
    {
      if(time_started>500)  time_started=false;
      yaw(yaw_min_speed);
      pitch(halfsecondcounter);  //necesitamos una funcion -x^2 <---------------------------------------------*****agasdgasd****
      halfsecondcounter++;
    break;
    }
    case 11:
    {
    break;
    }
    case 12:
    {
    break;
    }
    case 13:
    {
    break;
    }
  }
}

void setup()
{
	for(int i=0; i<4; i++) pinMode(motor_pin[i], OUTPUT);
	
		//{Acc  config BEGIN}
		#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
			Wire.begin();
			TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
		#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
			Fastwire::setup(400, true);
		#endif

    if(debug){Serial.begin(9600); sleep(8000);}
    
		accelgyro.initialize();
		devStatus = accelgyro.dmpInitialize();
    if(debug && devStatus!=0) Serial.println("Error iniciando dmp (mpu6050)");
		accelgyro.setDMPEnabled(true);
		#ifdef PROMICRO
			attachInterrupt(4, dmpDataReady, RISING);	//interrupt 4 -> pin7 for pro micro
		#else 
			attachInterrupt(0, dmpDataReady, RISING);	//interrupt 0 -> pin2 for uno(also pro micro but it uses pin2 for sda)
		#endif
		mpuIntStatus = accelgyro.getIntStatus();
		dmpReady = true;
		packetSize = accelgyro.dmpGetFIFOPacketSize();
	//{Acc  config END}
	//{Mirf config BEGIN}
#ifdef USE_RF24

	#ifdef PROMICRO
		Mirf.cePin = 4;//7; lo cambiamos por el 4 para poder usar el 7 como interrupcion en PROMICRO
    Mirf.csnPin = 8;//A0; //lo ponemos en A0 (pin18-A0) para tener mejor ordenados los cables junto a sck,miso,mosi
	#else
		Mirf.cePin = 7;
    Mirf.csnPin = 8;
	#endif
	
		Mirf.spi = &MirfHardwareSpi;
		Mirf.init();
		Mirf.setRADDR((byte *)"srv1");
		Mirf.payload = sizeof(paquetes);
		Mirf.channel = 19; //
		Mirf.configRegister( RF_SETUP, ( 1<<2 | 1<<1 ) ); //+power
		Mirf.config();
#endif		
	//{Mirf config END}
	
	
	#ifdef	USE_MS5611	//si usamos barometro para estabilizar eje z
		if(debug){
			while(!ms5611.begin())  Serial.println("Could not find a valid MS5611 sensor, check wiring!");
      Serial.println("MS5611 presure sensor detected!");
    }
		referencePressure = ms5611.readPressure();
		if(debug)	Serial.println(ms5611.getOversampling());
		
		get_width=&F_MS5611;	//metemos la funcion en una variable puntero
	#endif
	
	#ifdef	USE_HCSR04	//si usamos sensor de ultrasonidos para estabilizar eje z
		pinMode(trigPin, OUTPUT); //trigPin (18 is a0 in proMicro)
		pinMode(echoPin, INPUT);  //echoPin (19 is a1 in proMicro)
		
		get_width=&F_HCSR04;	//metemos la funcion en una variable puntero
	#endif
	
	if(debug)
	{
		Serial.println("Testing device connections...");
		Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
		Serial.print("Altura relativa:"); Serial.print( get_width() ); Serial.print(" cm.\n");
		Serial.print("Temperatura:"); Serial.print( ms5611.readTemperature() ); Serial.print(" *C\n");		
	}
}

void loop()
{
	stable_z=init_stable_z;
    if (!dmpReady) {
      if(debug) Serial.println("Accelerometer failed");
      return; //(acc) if programming failed, don't try to do anything
    }
	//boolean use_acc=true;
	//boolean use_gyro=false;
	boolean REPLY=false;
  int p1=0,p2=0; //parametros recibidos del modulo inalambrico (motor/speed)
  int flag_preprogrammed_flight=0;
  int z_min,z_max;
  int second_counter=0;
	boolean timerZenabled=false;
  boolean flag_stabilize_z=false;
  boolean bitAnotherSecond=false;
  byte byte250div=0,byte250measures=0;  //uno cuenta divisiones de 250ms hasta 1s, y el otro hasta 8veces 250ms
	
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
				if(!is_stable_xy()) //si no esta estable arranca	//de momento solo mira la estabilidad de XY, la estabilidad de z habría que hacer medidas y mirar la diff de altura
				{
					if(debug){
						Serial.print("Arrancamos por inestabilidad: ");
						Serial.print("grados y = ");  Serial.print(y); Serial.print(" / grados x = ");  Serial.print(x); /*Serial.print(" / altitud z = "); Serial.print(z);*/ Serial.print("\n");
					}
					time_started=false;//paramos el time_started porque lo vamos a reutilizar despues.
					break;
				}
		}
		REPLY=get_command(&p1,&p2);
	}

	//*motors activated
		power_all(starting_speed,true);
		sleep(400);

	//*stabilising xyz
		stabilize_xyz(no_stable_xy_strict);
		if(debug)Serial.println("Dron estabilizado en xyz");
	//once stabilised wait for orders and stabilize again:
	do{
		REPLY=get_command(&p1,&p2);
		if(REPLY)
		{
			//Limpiamos flags, temporizadores y contadores
			time_started=false;
			second_counter=0;
			flag_stabilize_z=false;//si recibimos respuesta dejamos de estabilizar eje z.
			timerZenabled=false;//si recibimos respuesta dejamos de MEDIR PARA estabilizar eje z.
			byte250div=0;
			byte250measures=0;
			bitAnotherSecond=false;
			
			
			switch(p1)	//ordenes que no sean de mover un motor
			{
				case 1: roll(p2);		//(+) se mueve en el eje x
				break;
				case 2: pitch(p2);		//(+) se mueve en el eje y
				break;
				case 3: power_all(p2);	// aumenta o reduce vel. de todos los motores
				break;
				case 4:	// gira sobre si mismo
				{
					switch(p2)
					{
						case 0 :
							yaw(-yaw_speed);		
						break;
						case 1 :
							yaw(yaw_speed);		
						break;
						case 10 :
						case 11 :
						case 12 :
						case 13 :
							flag_preprogrammed_flight=p2;
						break;
						default :
							if(debug)Serial.println("ERROR case doesnt exist");
							//flag_preprogrammed_flight=0; <- no es necesario ya que el flag no va a cambiar
						break;
					}
					break;
				}
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
		
		if(time_diff<ms_to_strict_stabilize)
			stabilize_xy();//stabilize dron with {no_stable_xy_limit} 
		else //If not receive command in 1 seconds: stabilize drone with {no_stable_xy_strict}
		{
			for(int i=0;i<3;i++)	speed[i]=stable[i];
			stabilize_xy(no_stable_xy_strict);
			time_started=false;
			//time_start = millis(); <- al haber desactivado time_started no es necesario reiniciar time_start
			second_counter++; //contamos los segundos en los que no se le ha mandado comandos.
			timerZenabled=true;//contador que mide cada 250ms altura para ver si z es estable
		}

		if(timerZenabled && time_diff/250>byte250div)//cada 0.25s medimos la altura
		{//lo hemos hecho de esta forma ya que probablamente time_diff no mida exactamente en el milisegundo 250,500,750,1000... habrá algunos ms que se salte mientras ejecuta otras funciones
			if(byte250measures==0)	//la primera medida la almacenamos como valor de altura al que estabilizar (queremos que se mantenga ahí)
			{
				stable_z=get_width();
				z_min=stable_z;
				z_max=stable_z;
			}
			else
			{
				z_aux=get_width();
				if(z_aux<z_min) z_min=z_aux;
				else z_max=z_aux;
			}
			byte250div++;
			if(byte250div==4)	byte250div=0;	//reiniciamos el byte de divisiones de 250ms ya que solo mide hasta 1segundo.
			byte250measures++;
			
			if(byte250measures==8)	//hemos hecho 8 medidas (8*250=2segundos) si la altura ha cambiado más de 70cm estabilizamos altura
			{
				timerZenabled=false;
				byte250measures=0;
				byte250div=0; //aunque ya se puso a 0 en if(byte250div==4)
				if( stable_z>z_min+Zmargin || stable_z<z_max-Zmargin)	//estabilizamos si no está entre los márgenes
					flag_stabilize_z=true;
			}
		}

		if(flag_stabilize_z  && time_diff/250>byte250div)	//reutilizamos el contador de 250divisiones
		{	//estabilizamos z 8? veces (una cada 250ms).
			stabilize_z();
			timerZenabled=false;//evitamos que intente activar el contador de estabilización z ya que ya se está estabilizando
			byte250div++;
			if(byte250div==4 && !bitAnotherSecond)
			{
				byte250div=0;
				bitAnotherSecond=true;
			}
			if(byte250div==4 && bitAnotherSecond)	//hemos medido 2 * 4 veces (ya hemos estabilizado el dron 8 veces)
			{
				for(int i=0;i<3;i++)
					stable[i]=speed[i];
				flag_stabilize_z=false;
				byte250div=0;
				bitAnotherSecond=false;
			}
		}
		
		if(flag_preprogrammed_flight>0)preprogrammed_flight_function(flag_preprogrammed_flight);

		if(second_counter>120) stop();//CONTAR 120 SEGUNDOS Y VOLVER A LA BASE <- de momento baja y se apaga
	}while(p1!=100);
	
	sleep(3000);
}

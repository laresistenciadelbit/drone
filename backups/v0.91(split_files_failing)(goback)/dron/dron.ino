// 0 .__. 1    <--motors
//   |  |
// 2 .__. 3 
// * <- significa que falta codigo

/*#define USE_HC-SR04*/ //si usamos sensor de distancia para medir eje Z
#define USE_MS5611    //si usamos barometro MS5611    para medir eje Z

#include <Wire.h>  //para acc y ms5611

//Acelerometro
  #include "I2Cdev.h" //para usar el acelerometro
  #include "MPU6050_6Axis_MotionApps20.h" //usamos DMP tambien
  //
  MPU6050 accelgyro;
  //int16_t ax, ay, az;
  //int16_t gx, gy, gz;
  #include "dmp_vars.h"

  
 //Mirf
  #include <SPI.h>
  #include <Mirf.h>
  #include <Mirf_nRF24L01.h>
  #include <MirfHardwareSpiDriver.h>

  
//barometro
#ifdef  USE_MS5611
  #include <MS5611.h>
  MS5611 ms5611;
  double referencePressure;
#endif

const boolean debug=false;
const int motor_pin[4]={5,6,9,10}; //pines de los motores //meter en setup?
int speed[4]={0,0,0,0};//vector de velocidades
int stable[4]={200,200,200,200};//vector de velocidad estable (200 de inicial por poner algo)
int x,y,z=0/*,gx,gy,gz*/, x_aux=0,y_aux=0,z_aux=0; //para no tener que crearlas cada vez que entra en get_position // x e y son grados, z es aceleracion (ya no se usa gy,gx y todo es del dmp)

const int starting_speed=220; //0-255 / es un valor sin medir, luego se estabilizara el solo.
const int regulable_speed=10; //suma o resta velocidad en tramos de regulable_speed (10) para calibrar z (que sea divisible entre 2)

const float Xoffset=0,Yoffset=2.75; //calibrado a ojo (a veces da 2.75, a veces 3.5)
const int no_stable_z=2000;//a partir de este valor la altura no esta estabilizada
/**/const int stablishing_z_limit=200; //velocidad limite de subida para autoestabilizacion
const int no_stable_xy_strict=8;  //por debajo o encima de este numero el eje x o y no esta estabilizado (mientras no se mueve)
const int no_stable_xy_limit=45; //estabilizacion menos estricta (durante el vuelo)
const boolean AUTO_BOOT_WHEN_UNSTABLE=true; //si se activa, al arrancar si no recive comandos, cada medio segundo comprueba si esta estable, sino, arranca

unsigned long time, last_t;

#include "header.h" //cabecera de funciones
#include "MPU_functions.cpp" //funciones que llaman a mpu6050
#include "functions.cpp"

void setup()
{
	for(int i=0; i<4; i++) pinMode(motor_pin[i], OUTPUT);
	
	#ifdef	USE_MS5611	//si usamos barometro para estabilizar eje z
		if(debug)
			while(!ms5611.begin())
				Serial.println("Could not find a valid MS5611 sensor, check wiring!");
		referencePressure = ms5611.readPressure();
		if(debug)	Serial.println(ms5611.getOversampling());
	#endif
	
	#ifdef	USE_HC-SR04	//si usamos sensor de ultrasonidos para estabilizar eje z
		pinMode(18, OUTPUT); //trigPin (18 is a0 in proMicro)
		pinMode(19, INPUT);  //echoPin (19 is a1 in proMicro)
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
		Mirf.cePin = 7;
		Mirf.csnPin = 8;
		Mirf.spi = &MirfHardwareSpi;
		Mirf.init();
		Mirf.setRADDR((byte *)"srv1");
		Mirf.payload = sizeof(paquetes);
		Mirf.channel = 19; //
		Mirf.configRegister( RF_SETUP, ( 1<<2 | 1<<1 ) ); //+power
		Mirf.config();
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
		power_all(starting_speed);
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

// 0 .__. 1    <--motors
//   |  |
// 2 .__. 3 
// * <- significa que falta codigo

const boolean debug=false;

 //Mirf
#include <SPI.h>
#include <Mirf.h>
#include <Mirf_nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
 //Acelerometro
#include "I2Cdev.h" //para usar el acelerometro
#include "MPU6050_6Axis_MotionApps20.h" //usamos DMP tambien
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 accelgyro;
//int16_t ax, ay, az;
//int16_t gx, gy, gz;
#include "dmp_vars.h"

const int motor_pin[4]={5,6,9,10}; //pines de los motores //meter en setup?
int speed[4]={0,0,0,0};//vector de velocidades
int x,y,z=0/*,gx,gy,gz*/, x_aux=0,y_aux=0,z_aux=0; //para no tener que crearlas cada vez que entra en get_position // x e y son grados, z es aceleracion (ya no se usa gy,gx y todo es del dmp)

const int starting_speed=220; //0-255 / es un valor sin medir, luego se estabilizara el solo.
const int regulable_speed=10; //suma o resta velocidad en tramos de regulable_speed (10) para calibrar z (que sea divisible entre 2)

const float Xoffset=0,Yoffset=2.75; //calibrado a ojo (a veces da 2.75, a veces 3.5)
const int no_stable_z=2000;//a partir de este valor la altura no esta estabilizada
const int no_stable_xy_strict=8;  //por debajo o encima de este numero el eje x o y no esta estabilizado (mientras no se mueve)
const int no_stable_xy_limit=45; //estabilizacion menos estricta (durante el vuelo)
const boolean AUTO_BOOT_WHEN_UNSTABLE=true; //si se activa, al arrancar si no recive comandos, cada medio segundo comprueba si esta estable, sino, arranca

unsigned long time, last_t;
void sleep(unsigned int delay_t)
{
	time=millis();
	last_t=time;
	while( last_t<(time+delay_t) )
		last_t=millis();
}

int stabilize_z()	//usamos giroscopio (ahora usamos cambios de gyro en x e y si los angulos son estables) +/-2000
{
	//do{
		z=get_z_acc();
		
		if(z < -no_stable_z)
			power_all(+regulable_speed);
		else if(z > no_stable_z) 
			power_all(-regulable_speed);
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
}

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
            //z = ypr[2] * 180/M_PI; //quitamos z ya que no nos interesa la informacion de los grados en ese eje para la estabilizacion
    }
}

boolean is_stable()
{
	z=get_z_acc();
	get_angle();
	if (y>Yoffset+no_stable_xy_limit || y<Yoffset-no_stable_xy_limit || x>Xoffset+no_stable_xy_limit || x<Xoffset-no_stable_xy_limit || z< -no_stable_z || z>no_stable_z) //aumentamos el limite +-100 en eje z
		 return(false);
	else return(true);
}

void stabilize_xy(int no_stable_xy=45)
{
	//do{
		get_angle();
		if(y>Yoffset+no_stable_xy || y<Yoffset-no_stable_xy) pitch(regulable_speed);
		if(x>Xoffset+no_stable_xy || x<Xoffset-no_stable_xy)  roll(regulable_speed);
	//}while(y>Yoffset+no_stable_xy || y<Yoffset-no_stable_xy || x>Xoffset+no_stable_xy || x<Xoffset-no_stable_xy);
}

void stabilize_xyz(int no_stable_xy=45)
{
	do{
		stabilize_xy(no_stable_xy);
		stabilize_z();
	}while( (y>Yoffset+no_stable_xy) || (y<Yoffset-no_stable_xy) || (x>Xoffset+no_stable_xy) || (x<Xoffset-no_stable_xy) || (z< -no_stable_z) || (z>no_stable_z) );
}

void power(int motor_number, int add)
{ //add speed to a motor				//we verify that it doesnt over speed it
	if(speed[motor_number]+add>255)
		speed[motor_number]=255;
	else
		speed[motor_number]+=add;
		
	analogWrite(motor_pin[motor_number], speed[motor_number]);
}
void power_all(int add)
{ //add speed to all motors				//we verify that it doesnt over speed it
	for(int i=0;i<4;i++){
		if(speed[i]+add>255)	speed[i]=255;
		else 					speed[i]+=add;
		analogWrite(motor_pin[i],speed[i]);
	}
}

void stop()			/////////////si al apagar por ejemplo, se para antes de tiempo, significa que ha obtenido un valor de 
{
	stabilize_xyz(no_stable_xy_strict); //estabilizamos primero
	sleep(100);	//esperamos por si los valores de z no son estables aun
	do{
		z=get_z_acc();
		power_all(-regulable_speed);
		sleep(500);
	}while(z>no_stable_z || speed[0]+speed[1]+speed[2]+speed[3]<40);	//si no hemos mapeado, por debajo de 80 probablemente no se muevan los motores. Asi que se sale si los motores se han parado ya y el acc no ha medido bien, o no ha llegado al suelo y se ha parado.
}

struct paquetes {
   unsigned long  time;
   int c; //command
   int s; //subcommand
} paquete;

boolean get_command(int *p1, int *p2) //recive los comandos por el modulo inalambrico.
{
	boolean REPLY=false;
	
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

	*p1=paquete.c;
	*p2=paquete.s;
	
	if (REPLY) return(true);
	else return(false);
}

void roll(int add) //movimiento del eje x
{
	 speed[1]-=(add/2);	//escribimos la nueva velocidad en el vector
	 speed[3]+=(add/2);
	power(1,speed[1]); //quitamos mitad de velocidad de la pulsada al motor 3 (para valores positivos)
	power(3,speed[3]); //damos mitad de velocidad de la pulsada al motor 1    (para valores positivos)
}

void pitch(int add)  //movimiento del eje y
{
	 speed[0]-=(add/2);
	 speed[2]+=(add/2);
	power(0,speed[0]);
	power(2,speed[2]);
}

void yaw(int add) //movimiento de giro sobre si mismo (z==gravedad) (usar l1,r1)
{
	 speed[0]-=(add/2);
	 speed[2]-=(add/2);
	 speed[1]+=(add/2);
	 speed[3]+=(add/2);
	power(0,speed[0]);
	power(2,speed[2]);
	power(1,speed[1]);
	power(3,speed[3]);
}

void motor_x(int add) //reduce la velocidad del eje_x del dron para moverse hacia ese lado
{
	 speed[0]-=(add/2);
	 speed[3]-=(add/2);
	 speed[1]+=(add/2);
	 speed[2]+=(add/2);
	power(0,speed[0]);
	power(3,speed[3]);
	power(1,speed[1]);
	power(2,speed[2]);
}
void motor_y(int add) //reduce la velocidad del eje_y del dron para moverse hacia ese lado
{
	 speed[0]-=(add/2);
	 speed[1]-=(add/2);
	 speed[2]+=(add/2);
	 speed[3]+=(add/2);
	power(0,speed[0]);
	power(1,speed[1]);
	power(2,speed[2]);
	power(3,speed[3]);
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
			
			second_counter++; //contamos los segundos en los que no se le ha mandado comandos.
		}
		//stabilize_z(); called in stabilize_xyz();
		
		if(second_counter>60) stop();//CONTAR 60 SEGUNDOS Y VOLVER A LA BASE <- de momento baja y se apaga
	}while(p1!=100);
	
	sleep(3000);

}
